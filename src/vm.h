#pragma once

#include "common.h"
#include "frame.h"
#include "error.h"
#include "gc.h"

namespace pkpy{

#define DEF_NATIVE_2(ctype, ptype)                                      \
    template<> ctype py_cast<ctype>(VM* vm, PyObject* obj) {            \
        vm->check_type(obj, vm->ptype);                                 \
        return OBJ_GET(ctype, obj);                                     \
    }                                                                   \
    template<> ctype _py_cast<ctype>(VM* vm, PyObject* obj) {           \
        return OBJ_GET(ctype, obj);                                     \
    }                                                                   \
    template<> ctype& py_cast<ctype&>(VM* vm, PyObject* obj) {          \
        vm->check_type(obj, vm->ptype);                                 \
        return OBJ_GET(ctype, obj);                                     \
    }                                                                   \
    template<> ctype& _py_cast<ctype&>(VM* vm, PyObject* obj) {         \
        return OBJ_GET(ctype, obj);                                     \
    }                                                                   \
    PyObject* py_var(VM* vm, const ctype& value) { return vm->heap.gcnew(vm->ptype, value);}     \
    PyObject* py_var(VM* vm, ctype&& value) { return vm->heap.gcnew(vm->ptype, std::move(value));}

class Generator: public BaseIter {
    std::unique_ptr<Frame> frame;
    int state; // 0,1,2
public:
    Generator(VM* vm, std::unique_ptr<Frame>&& frame)
        : BaseIter(vm, nullptr), frame(std::move(frame)), state(0) {}

    PyObject* next();
};

struct PyTypeInfo{
    PyObject* obj;
    Type base;
    Str name;
};

class VM {
    VM* vm;     // self reference for simplify code
    ManagedHeap heap;
public:
    std::stack< std::unique_ptr<Frame> > callstack;
    std::vector<PyTypeInfo> _all_types;

    PyObject* run_frame(Frame* frame);

    NameDict _modules;                          // loaded modules
    std::map<StrName, Str> _lazy_modules;       // lazy loaded modules

    // singleton objects, need_gc=false
    PyObject* _py_op_call;
    PyObject* _py_op_yield;
    PyObject* None;
    PyObject* True;
    PyObject* False;
    PyObject* Ellipsis;

    PyObject* builtins;         // builtins module
    PyObject* _main;            // __main__ module

    bool use_stdio;
    std::ostream* _stdout;
    std::ostream* _stderr;
    int recursionlimit = 1000;

    VM(bool use_stdio){
        this->vm = this;
        this->use_stdio = use_stdio;
        if(use_stdio){
            this->_stdout = &std::cout;
            this->_stderr = &std::cerr;
        }else{
            this->_stdout = new StrStream();
            this->_stderr = new StrStream();
        }

        init_builtin_types();
    }

    PyObject* asStr(PyObject* obj){
        PyObject* f = getattr(obj, __str__, false, true);
        if(f != nullptr) return call(f);
        return asRepr(obj);
    }

    inline Frame* top_frame() const {
#if PK_EXTRA_CHECK
        if(callstack.empty()) UNREACHABLE();
#endif
        return callstack.top().get();
    }

    PyObject* asIter(PyObject* obj){
        if(is_type(obj, tp_iterator)) return obj;
        PyObject* iter_f = getattr(obj, __iter__, false, true);
        if(iter_f != nullptr) return call(iter_f);
        TypeError(OBJ_NAME(_t(obj)).escape(true) + " object is not iterable");
        return nullptr;
    }

    PyObject* asList(PyObject* iterable){
        if(is_type(iterable, tp_list)) return iterable;
        return call(_t(tp_list), Args{iterable});
    }

    PyObject** find_name_in_mro(PyObject* cls, StrName name){
        PyObject** val;
        do{
            val = cls->attr().try_get(name);
            if(val != nullptr) return val;
            Type cls_t = static_cast<Py_<Type>*>(cls)->_value;
            Type base = _all_types[cls_t.index].base;
            if(base.index == -1) break;
            cls = _all_types[base.index].obj;
        }while(true);
        return nullptr;
    }

    bool isinstance(PyObject* obj, Type cls_t){
        Type obj_t = OBJ_GET(Type, _t(obj));
        do{
            if(obj_t == cls_t) return true;
            Type base = _all_types[obj_t.index].base;
            if(base.index == -1) break;
            obj_t = base;
        }while(true);
        return false;
    }

    PyObject* fast_call(StrName name, Args&& args){
        PyObject** val = find_name_in_mro(_t(args[0]).get(), name);
        if(val != nullptr) return call(*val, std::move(args));
        AttributeError(args[0], name);
        return nullptr;
    }

    inline PyObject* call(PyObject* _callable){
        return call(_callable, no_arg(), no_arg(), false);
    }

    template<typename ArgT>
    inline std::enable_if_t<std::is_same_v<std::decay_t<ArgT>, Args>, PyObject*>
    call(PyObject* _callable, ArgT&& args){
        return call(_callable, std::forward<ArgT>(args), no_arg(), false);
    }

    template<typename ArgT>
    inline std::enable_if_t<std::is_same_v<std::decay_t<ArgT>, Args>, PyObject*>
    call(PyObject* obj, const StrName name, ArgT&& args){
        return call(getattr(obj, name, true, true), std::forward<ArgT>(args), no_arg(), false);
    }

    inline PyObject* call(PyObject* obj, StrName name){
        return call(getattr(obj, name, true, true), no_arg(), no_arg(), false);
    }


    // repl mode is only for setting `frame->id` to 0
    PyObject* exec(Str source, Str filename, CompileMode mode, PyObject* _module=nullptr){
        if(_module == nullptr) _module = _main;
        try {
            CodeObject_ code = compile(source, filename, mode);
            return _exec(code, _module);
        }catch (const Exception& e){
            *_stderr << e.summary() << '\n';
        }catch (const std::exception& e) {
            *_stderr << "An std::exception occurred! It could be a bug.\n";
            *_stderr << e.what() << '\n';
        }
        callstack = {};
        return nullptr;
    }

    template<typename ...Args>
    inline std::unique_ptr<Frame> _new_frame(Args&&... args){
        if(callstack.size() > recursionlimit){
            _error("RecursionError", "maximum recursion depth exceeded");
        }
        return std::make_unique<Frame>(std::forward<Args>(args)...);
    }

    template<typename ...Args>
    inline PyObject* _exec(Args&&... args){
        callstack.push(_new_frame(std::forward<Args>(args)...));
        return _exec();
    }

    PyObject* property(NativeFuncRaw fget){
        PyObject* p = builtins->attr("property");
        PyObject* method = heap.gcnew(tp_native_function, NativeFunc(fget, 1, false));
        return call(p, Args{method});
    }

    PyObject* new_type_object(PyObject* mod, StrName name, Type base){
        // use gcnew
        PyObject* obj = new Py_<Type>(tp_type, _all_types.size());
        PyTypeInfo info{
            .obj = obj,
            .base = base,
            .name = (mod!=nullptr && mod!=builtins) ? Str(OBJ_NAME(mod)+"."+name.str()): name.str()
        };
        if(mod != nullptr) mod->attr().set(name, obj);
        _all_types.push_back(info);
        return obj;
    }

    Type _new_type_object(StrName name, Type base=0) {
        PyObject* obj = new_type_object(nullptr, name, base);
        return OBJ_GET(Type, obj);
    }

    PyObject* _find_type(const Str& type){
        PyObject** obj = builtins->attr().try_get(type);
        if(!obj){
            for(auto& t: _all_types) if(t.name == type) return t.obj;
            throw std::runtime_error("type not found: " + type);
        }
        return *obj;
    }

    template<int ARGC>
    void bind_func(Str type, Str name, NativeFuncRaw fn) {
        bind_func<ARGC>(_find_type(type), name, fn);
    }

    template<int ARGC>
    void bind_method(Str type, Str name, NativeFuncRaw fn) {
        bind_method<ARGC>(_find_type(type), name, fn);
    }

    template<int ARGC, typename... Args>
    void bind_static_method(Args&&... args) {
        bind_func<ARGC>(std::forward<Args>(args)...);
    }

    template<int ARGC>
    void _bind_methods(std::vector<Str> types, Str name, NativeFuncRaw fn) {
        for(auto& type: types) bind_method<ARGC>(type, name, fn);
    }

    template<int ARGC>
    void bind_builtin_func(Str name, NativeFuncRaw fn) {
        bind_func<ARGC>(builtins, name, fn);
    }

    int normalized_index(int index, int size){
        if(index < 0) index += size;
        if(index < 0 || index >= size){
            IndexError(std::to_string(index) + " not in [0, " + std::to_string(size) + ")");
        }
        return index;
    }

    // for quick access
    Type tp_object, tp_type, tp_int, tp_float, tp_bool, tp_str;
    Type tp_list, tp_tuple;
    Type tp_function, tp_native_function, tp_iterator, tp_bound_method;
    Type tp_slice, tp_range, tp_module, tp_ref;
    Type tp_super, tp_exception, tp_star_wrapper;

    template<typename P>
    inline PyObject* PyIter(P&& value) {
        static_assert(std::is_base_of_v<BaseIter, std::decay_t<P>>);
        return heap.gcnew<P>(tp_iterator, std::forward<P>(value));
    }

    inline BaseIter* PyIter_AS_C(PyObject* obj)
    {
        check_type(obj, tp_iterator);
        return static_cast<BaseIter*>(obj->value());
    }
    
    /***** Error Reporter *****/
    void _error(StrName name, const Str& msg){
        _error(Exception(name, msg));
    }

    void _raise(){
        bool ok = top_frame()->jump_to_exception_handler();
        if(ok) throw HandledException();
        else throw UnhandledException();
    }

public:
    void IOError(const Str& msg) { _error("IOError", msg); }
    void NotImplementedError(){ _error("NotImplementedError", ""); }
    void TypeError(const Str& msg){ _error("TypeError", msg); }
    void ZeroDivisionError(){ _error("ZeroDivisionError", "division by zero"); }
    void IndexError(const Str& msg){ _error("IndexError", msg); }
    void ValueError(const Str& msg){ _error("ValueError", msg); }
    void NameError(StrName name){ _error("NameError", "name " + name.str().escape(true) + " is not defined"); }

    void AttributeError(PyObject* obj, StrName name){
        _error("AttributeError", "type " +  OBJ_NAME(_t(obj)).escape(true) + " has no attribute " + name.str().escape(true));
    }

    void AttributeError(Str msg){ _error("AttributeError", msg); }

    inline void check_type(PyObject* obj, Type type){
        if(is_type(obj, type)) return;
        TypeError("expected " + OBJ_NAME(_t(type)).escape(true) + ", but got " + OBJ_NAME(_t(obj)).escape(true));
    }

    inline PyObject* _t(Type t){
        return _all_types[t.index].obj;
    }

    inline PyObject* _t(PyObject* obj){
        if(is_int(obj)) return _t(tp_int);
        if(is_float(obj)) return _t(tp_float);
        return _all_types[OBJ_GET(Type, _t(obj->type)).index].obj;
    }

    ~VM() {
        if(!use_stdio){
            delete _stdout;
            delete _stderr;
        }
    }

    CodeObject_ compile(Str source, Str filename, CompileMode mode);
    void post_init();
    PyObject* num_negated(PyObject* obj);
    f64 num_to_float(PyObject* obj);
    PyObject* asBool(PyObject* obj);
    i64 hash(PyObject* obj);
    PyObject* asRepr(PyObject* obj);
    PyObject* new_module(StrName name);
    Str disassemble(CodeObject_ co);
    void init_builtin_types();
    PyObject* call(PyObject* _callable, Args args, const Args& kwargs, bool opCall);
    void unpack_args(Args& args);
    PyObject* getattr(PyObject* obj, StrName name, bool throw_err=true, bool class_only=false);
    template<typename T>
    void setattr(PyObject* obj, StrName name, T&& value);
    template<int ARGC>
    void bind_method(PyObject* obj, Str funcName, NativeFuncRaw fn);
    template<int ARGC>
    void bind_func(PyObject* obj, Str funcName, NativeFuncRaw fn);
    void _error(Exception e);
    PyObject* _exec();

    template<typename P>
    PyObject* PyRef(P&& value);
    const BaseRef* PyRef_AS_C(PyObject* obj);
};

PyObject* NativeFunc::operator()(VM* vm, Args& args) const{
    int args_size = args.size() - (int)method;  // remove self
    if(argc != -1 && args_size != argc) {
        vm->TypeError("expected " + std::to_string(argc) + " arguments, but got " + std::to_string(args_size));
    }
    return f(vm, args);
}

void CodeObject::optimize(VM* vm){
    std::vector<StrName> keys;
    for(auto& p: names) if(p.second == NAME_LOCAL) keys.push_back(p.first);
    uint32_t base_n = (uint32_t)(keys.size() / kLocalsLoadFactor + 0.5);
    perfect_locals_capacity = find_next_capacity(base_n);
    perfect_hash_seed = find_perfect_hash_seed(perfect_locals_capacity, keys);

    for(int i=1; i<codes.size(); i++){
        if(codes[i].op == OP_UNARY_NEGATIVE && codes[i-1].op == OP_LOAD_CONST){
            codes[i].op = OP_NO_OP;
            int pos = codes[i-1].arg;
            consts[pos] = vm->num_negated(consts[pos]);
        }

        if(i>=2 && codes[i].op == OP_BUILD_INDEX){
            const Bytecode& a = codes[i-1];
            const Bytecode& x = codes[i-2];
            if(codes[i].arg == 1){
                if(a.op == OP_LOAD_NAME && x.op == OP_LOAD_NAME){
                    codes[i].op = OP_FAST_INDEX;
                }else continue;
            }else{
                if(a.op == OP_LOAD_NAME_REF && x.op == OP_LOAD_NAME_REF){
                    codes[i].op = OP_FAST_INDEX_REF;
                }else continue;
            }
            codes[i].arg = (a.arg << 16) | x.arg;
            codes[i-1].op = OP_NO_OP;
            codes[i-2].op = OP_NO_OP;
        }
    }

    // pre-compute sn in co_consts
    for(int i=0; i<consts.size(); i++){
        if(is_type(consts[i], vm->tp_str)){
            Str& s = OBJ_GET(Str, consts[i]);
            s._cached_sn_index = StrName::get(s.c_str()).index;
        }
    }
}

DEF_NATIVE_2(Str, tp_str)
DEF_NATIVE_2(List, tp_list)
DEF_NATIVE_2(Tuple, tp_tuple)
DEF_NATIVE_2(Function, tp_function)
DEF_NATIVE_2(NativeFunc, tp_native_function)
DEF_NATIVE_2(BoundMethod, tp_bound_method)
DEF_NATIVE_2(Range, tp_range)
DEF_NATIVE_2(Slice, tp_slice)
DEF_NATIVE_2(Exception, tp_exception)
DEF_NATIVE_2(StarWrapper, tp_star_wrapper)

#define PY_CAST_INT(T) \
template<> T py_cast<T>(VM* vm, PyObject* obj){ \
    vm->check_type(obj, vm->tp_int); \
    return (T)(obj.bits >> 2); \
} \
template<> T _py_cast<T>(VM* vm, PyObject* obj){ \
    return (T)(obj.bits >> 2); \
}

PY_CAST_INT(char)
PY_CAST_INT(short)
PY_CAST_INT(int)
PY_CAST_INT(long)
PY_CAST_INT(long long)
PY_CAST_INT(unsigned char)
PY_CAST_INT(unsigned short)
PY_CAST_INT(unsigned int)
PY_CAST_INT(unsigned long)
PY_CAST_INT(unsigned long long)


template<> float py_cast<float>(VM* vm, PyObject* obj){
    vm->check_type(obj, vm->tp_float);
    i64 bits = obj.bits;
    bits = (bits >> 2) << 2;
    return BitsCvt(bits)._float;
}
template<> float _py_cast<float>(VM* vm, PyObject* obj){
    i64 bits = obj.bits;
    bits = (bits >> 2) << 2;
    return BitsCvt(bits)._float;
}
template<> double py_cast<double>(VM* vm, PyObject* obj){
    vm->check_type(obj, vm->tp_float);
    i64 bits = obj.bits;
    bits = (bits >> 2) << 2;
    return BitsCvt(bits)._float;
}
template<> double _py_cast<double>(VM* vm, PyObject* obj){
    i64 bits = obj.bits;
    bits = (bits >> 2) << 2;
    return BitsCvt(bits)._float;
}


#define PY_VAR_INT(T)                           \
    PyObject* py_var(VM* vm, T _val){           \
        i64 val = static_cast<i64>(_val);       \
        if(((val << 2) >> 2) != val){           \
            vm->_error("OverflowError", std::to_string(val) + " is out of range");  \
        }                                                                           \
        val = (val << 2) | 0b01;                                                    \
        return reinterpret_cast<PyObject*>(val);                                    \
    }

PY_VAR_INT(char)
PY_VAR_INT(short)
PY_VAR_INT(int)
PY_VAR_INT(long)
PY_VAR_INT(long long)
PY_VAR_INT(unsigned char)
PY_VAR_INT(unsigned short)
PY_VAR_INT(unsigned int)
PY_VAR_INT(unsigned long)
PY_VAR_INT(unsigned long long)

#define PY_VAR_FLOAT(T)                             \
    PyObject* py_var(VM* vm, T _val){               \
        f64 val = static_cast<f64>(_val);           \
        i64 bits = BitsCvt(val)._int;                  \
        bits = (bits >> 2) << 2;                    \
        bits |= 0b10;                               \
        return reinterpret_cast<PyObject*>(bits);   \
    }

PY_VAR_FLOAT(float)
PY_VAR_FLOAT(double)

PyObject* py_var(VM* vm, bool val){
    return val ? vm->True : vm->False;
}

template<> bool py_cast<bool>(VM* vm, PyObject* obj){
    vm->check_type(obj, vm->tp_bool);
    return obj == vm->True;
}
template<> bool _py_cast<bool>(VM* vm, PyObject* obj){
    return obj == vm->True;
}

PyObject* py_var(VM* vm, const char val[]){
    return VAR(Str(val));
}

PyObject* py_var(VM* vm, std::string val){
    return VAR(Str(std::move(val)));
}

template<typename T>
void _check_py_class(VM* vm, PyObject* obj){
    vm->check_type(obj, T::_type(vm));
}

PyObject* VM::num_negated(PyObject* obj){
    if (is_int(obj)){
        return VAR(-CAST(i64, obj));
    }else if(is_float(obj)){
        return VAR(-CAST(f64, obj));
    }
    TypeError("expected 'int' or 'float', got " + OBJ_NAME(_t(obj)).escape(true));
    return nullptr;
}

f64 VM::num_to_float(PyObject* obj){
    if(is_float(obj)){
        return CAST(f64, obj);
    } else if (is_int(obj)){
        return (f64)CAST(i64, obj);
    }
    TypeError("expected 'int' or 'float', got " + OBJ_NAME(_t(obj)).escape(true));
    return 0;
}

PyObject* VM::asBool(PyObject* obj){
    if(is_type(obj, tp_bool)) return obj;
    if(obj == None) return False;
    if(is_type(obj, tp_int)) return VAR(CAST(i64, obj) != 0);
    if(is_type(obj, tp_float)) return VAR(CAST(f64, obj) != 0.0);
    PyObject* len_fn = getattr(obj, __len__, false, true);
    if(len_fn != nullptr){
        PyObject* ret = call(len_fn);
        return VAR(CAST(i64, ret) > 0);
    }
    return True;
}

i64 VM::hash(PyObject* obj){
    if (is_type(obj, tp_str)) return CAST(Str&, obj).hash();
    if (is_int(obj)) return CAST(i64, obj);
    if (is_type(obj, tp_tuple)) {
        i64 x = 1000003;
        const Tuple& items = CAST(Tuple&, obj);
        for (int i=0; i<items.size(); i++) {
            i64 y = hash(items[i]);
            x = x ^ (y + 0x9e3779b9 + (x << 6) + (x >> 2)); // recommended by Github Copilot
        }
        return x;
    }
    if (is_type(obj, tp_type)) return obj.bits;
    if (is_type(obj, tp_bool)) return _CAST(bool, obj) ? 1 : 0;
    if (is_float(obj)){
        f64 val = CAST(f64, obj);
        return (i64)std::hash<f64>()(val);
    }
    TypeError("unhashable type: " +  OBJ_NAME(_t(obj)).escape(true));
    return 0;
}

PyObject* VM::asRepr(PyObject* obj){
    return call(obj, __repr__);
}

PyObject* VM::new_module(StrName name) {
    PyObject* obj = new Py_<DummyModule>(tp_module, DummyModule());
    obj->attr().set(__name__, VAR(name.str()));
    // we do not allow override in order to avoid memory leak
    // it is because Module objects are not garbage collected
    if(_modules.contains(name)) UNREACHABLE();
    _modules.set(name, obj);
    return obj;
}

Str VM::disassemble(CodeObject_ co){
    std::vector<int> jumpTargets;
    for(auto byte : co->codes){
        if(byte.op == OP_JUMP_ABSOLUTE || byte.op == OP_SAFE_JUMP_ABSOLUTE || byte.op == OP_POP_JUMP_IF_FALSE){
            jumpTargets.push_back(byte.arg);
        }
    }
    StrStream ss;
    ss << std::string(54, '-') << '\n';
    ss << co->name << ":\n";
    int prev_line = -1;
    for(int i=0; i<co->codes.size(); i++){
        const Bytecode& byte = co->codes[i];
        if(byte.op == OP_NO_OP) continue;
        Str line = std::to_string(byte.line);
        if(byte.line == prev_line) line = "";
        else{
            if(prev_line != -1) ss << "\n";
            prev_line = byte.line;
        }

        std::string pointer;
        if(std::find(jumpTargets.begin(), jumpTargets.end(), i) != jumpTargets.end()){
            pointer = "-> ";
        }else{
            pointer = "   ";
        }
        ss << pad(line, 8) << pointer << pad(std::to_string(i), 3);
        ss << " " << pad(OP_NAMES[byte.op], 20) << " ";
        // ss << pad(byte.arg == -1 ? "" : std::to_string(byte.arg), 5);
        std::string argStr = byte.arg == -1 ? "" : std::to_string(byte.arg);
        if(byte.op == OP_LOAD_CONST){
            argStr += " (" + CAST(Str, asRepr(co->consts[byte.arg])) + ")";
        }
        if(byte.op == OP_LOAD_NAME_REF || byte.op == OP_LOAD_NAME || byte.op == OP_RAISE || byte.op == OP_STORE_NAME){
            argStr += " (" + co->names[byte.arg].first.str().escape(true) + ")";
        }
        if(byte.op == OP_FAST_INDEX || byte.op == OP_FAST_INDEX_REF){
            auto& a = co->names[byte.arg & 0xFFFF];
            auto& x = co->names[(byte.arg >> 16) & 0xFFFF];
            argStr += " (" + a.first.str() + '[' + x.first.str() + "])";
        }
        ss << pad(argStr, 20);      // may overflow
        ss << co->blocks[byte.block].to_string();
        if(i != co->codes.size() - 1) ss << '\n';
    }
    StrStream consts;
    consts << "co_consts: ";
    consts << CAST(Str, asRepr(VAR(co->consts)));

    StrStream names;
    names << "co_names: ";
    List list;
    for(int i=0; i<co->names.size(); i++){
        list.push_back(VAR(co->names[i].first.str()));
    }
    names << CAST(Str, asRepr(VAR(list)));
    ss << '\n' << consts.str() << '\n' << names.str() << '\n';

    for(int i=0; i<co->consts.size(); i++){
        PyObject* obj = co->consts[i];
        if(is_type(obj, tp_function)){
            const auto& f = CAST(Function&, obj);
            ss << disassemble(f.code);
        }
    }
    return Str(ss.str());
}

void VM::init_builtin_types(){
    PyObject* _tp_object = new Py_<Type>(Type(1), Type(0));
    PyObject* _tp_type = new Py_<Type>(Type(1), Type(1));
    // PyTypeObject is managed by _all_types
    // PyModuleObject is managed by _modules
    // They are not managed by GC, so we use a simple "new"
    _all_types.push_back({.obj = _tp_object, .base = -1, .name = "object"});
    _all_types.push_back({.obj = _tp_type, .base = 0, .name = "type"});
    tp_object = 0; tp_type = 1;

    tp_int = _new_type_object("int");
    tp_float = _new_type_object("float");
    if(tp_int.index != kTpIntIndex || tp_float.index != kTpFloatIndex) UNREACHABLE();

    tp_bool = _new_type_object("bool");
    tp_str = _new_type_object("str");
    tp_list = _new_type_object("list");
    tp_tuple = _new_type_object("tuple");
    tp_slice = _new_type_object("slice");
    tp_range = _new_type_object("range");
    tp_module = _new_type_object("module");
    tp_ref = _new_type_object("_ref");
    tp_star_wrapper = _new_type_object("_star_wrapper");
    
    tp_function = _new_type_object("function");
    tp_native_function = _new_type_object("native_function");
    tp_iterator = _new_type_object("iterator");
    tp_bound_method = _new_type_object("bound_method");
    tp_super = _new_type_object("super");
    tp_exception = _new_type_object("Exception");

    this->None = new Py_<Dummy>(_new_type_object("NoneType"), {});
    this->Ellipsis = new Py_<Dummy>(_new_type_object("ellipsis"), {});
    this->True = new Py_<Dummy>(tp_bool, {});
    this->False = new Py_<Dummy>(tp_bool, {});
    this->_py_op_call = new Py_<Dummy>(_new_type_object("_py_op_call"), {});
    this->_py_op_yield = new Py_<Dummy>(_new_type_object("_py_op_yield"), {});
    this->builtins = new_module("builtins");
    this->_main = new_module("__main__");
    
    // setup public types
    builtins->attr().set("type", _t(tp_type));
    builtins->attr().set("object", _t(tp_object));
    builtins->attr().set("bool", _t(tp_bool));
    builtins->attr().set("int", _t(tp_int));
    builtins->attr().set("float", _t(tp_float));
    builtins->attr().set("str", _t(tp_str));
    builtins->attr().set("list", _t(tp_list));
    builtins->attr().set("tuple", _t(tp_tuple));
    builtins->attr().set("range", _t(tp_range));

    post_init();
    for(int i=0; i<_all_types.size(); i++){
        auto& t = _all_types[i];
        t.obj->attr()._try_perfect_rehash();
    }
    for(auto [k, v]: _modules.items()) v->attr()._try_perfect_rehash();
}

PyObject* VM::call(PyObject* callable, Args args, const Args& kwargs, bool opCall){
    if(is_type(callable, tp_type)){
        PyObject** new_f = callable->attr().try_get(__new__);
        PyObject* obj;
        if(new_f != nullptr){
            obj = call(*new_f, std::move(args), kwargs, false);
        }else{
            obj = heap.gcnew<DummyInstance>(_callable, {});
            PyObject* init_f = getattr(obj, __init__, false, true);
            if (init_f != nullptr) call(init_f, std::move(args), kwargs, false);
        }
        return obj;
    }

    if(is_type(callable, tp_bound_method)){
        auto& bm = CAST(BoundMethod&, callable);
        callable = bm.method;      // get unbound method
        args.extend_self(bm.obj);
    }
    
    if(is_type(callable, tp_native_function)){
        const auto& f = OBJ_GET(NativeFunc, callable);
        if(kwargs.size() != 0) TypeError("native_function does not accept keyword arguments");
        return f(this, args);
    } else if(is_type(callable, tp_function)){
        const Function& fn = CAST(Function&, callable);
        NameDict_ locals = make_sp<NameDict>(
            fn.code->perfect_locals_capacity,
            kLocalsLoadFactor,
            fn.code->perfect_hash_seed
        );

        int i = 0;
        for(StrName name : fn.args){
            if(i < args.size()){
                locals->set(name, std::move(args[i++]));
                continue;
            }
            TypeError("missing positional argument " + name.str().escape(true));
        }

        locals->update(fn.kwargs);

        if(!fn.starred_arg.empty()){
            List vargs;        // handle *args
            while(i < args.size()) vargs.push_back(std::move(args[i++]));
            locals->set(fn.starred_arg, VAR(Tuple::from_list(std::move(vargs))));
        }else{
            for(StrName key : fn.kwargs_order){
                if(i < args.size()){
                    locals->set(key, std::move(args[i++]));
                }else{
                    break;
                }
            }
            if(i < args.size()) TypeError("too many arguments");
        }
        
        for(int i=0; i<kwargs.size(); i+=2){
            const Str& key = CAST(Str&, kwargs[i]);
            if(!fn.kwargs.contains(key)){
                TypeError(key.escape(true) + " is an invalid keyword argument for " + fn.name.str() + "()");
            }
            locals->set(key, kwargs[i+1]);
        }
        PyObject* _module = fn._module != nullptr ? fn._module : top_frame()->_module;
        auto _frame = _new_frame(fn.code, _module, locals, fn._closure);
        if(fn.code->is_generator) return PyIter(Generator(this, std::move(_frame)));
        callstack.push(std::move(_frame));
        if(opCall) return _py_op_call;
        return _exec();
    }

    PyObject* call_f = getattr(_callable, __call__, false, true);
    if(call_f != nullptr){
        return call(call_f, std::move(args), kwargs, false);
    }
    TypeError(OBJ_NAME(_t(*callable)).escape(true) + " object is not callable");
    return None;
}

void VM::unpack_args(Args& args){
    List unpacked;
    for(int i=0; i<args.size(); i++){
        if(is_type(args[i], tp_star_wrapper)){
            auto& star = _CAST(StarWrapper&, args[i]);
            if(!star.rvalue) UNREACHABLE();
            PyObject* list = asList(star.obj);
            List& list_c = CAST(List&, list);
            unpacked.insert(unpacked.end(), list_c.begin(), list_c.end());
        }else{
            unpacked.push_back(args[i]);
        }
    }
    args = Args::from_list(std::move(unpacked));
}

using Super = std::pair<PyObject*, Type>;

// https://docs.python.org/3/howto/descriptor.html#invocation-from-an-instance
PyObject* VM::getattr(PyObject* obj, StrName name, bool throw_err, bool class_only){
    PyObject* objtype = _t(obj);
    // handle super() proxy
    if(is_type(obj, tp_super)){
        const Super& super = OBJ_GET(Super, obj);
        obj = super.first;
        objtype = _t(super.second);
    }
    PyObject** cls_var = find_name_in_mro(objtype, name);
    if(cls_var != nullptr){
        // handle descriptor
        PyObject** descr_get = _t(*cls_var)->attr().try_get(__get__);
        if(descr_get != nullptr) return call(*descr_get, Args{*cls_var, obj});
    }
    // handle instance __dict__
    if(!class_only && !is_tagged(obj) && obj->is_attr_valid()){
        PyObject** val = obj->attr().try_get(name);
        if(val != nullptr) return *val;
    }
    if(cls_var != nullptr){
        // bound method is non-data descriptor
        if(is_type(*cls_var, tp_function) || is_type(*cls_var, tp_native_function)){
            return VAR(BoundMethod(obj, *cls_var));
        }
        return *cls_var;
    }
    if(throw_err) AttributeError(obj, name);
    return nullptr;
}

template<typename T>
void VM::setattr(PyObject* obj, StrName name, T&& value){
    static_assert(std::is_same_v<std::decay_t<T>, PyObject*>);
    PyObject* objtype = _t(obj);
    // handle super() proxy
    if(is_type(obj, tp_super)){
        Super& super = OBJ_GET(Super, *obj);
        obj = super.first;
        objtype = _t(super.second);
    }
    PyObject** cls_var = find_name_in_mro(objtype, name);
    if(cls_var != nullptr){
        // handle descriptor
        PyObject* cls_var_t = _t(*cls_var);
        if(cls_var_t->attr().contains(__get__)){
            PyObject** descr_set = cls_var_t->attr().try_get(__set__);
            if(descr_set != nullptr){
                call(*descr_set, Args{*cls_var, obj, std::forward<T>(value)});
            }else{
                TypeError("readonly attribute: " + name.str().escape(true));
            }
            return;
        }
    }
    // handle instance __dict__
    if(is_tagged(obj) || !obj->is_attr_valid()) TypeError("cannot set attribute");
    obj->attr().set(name, std::forward<T>(value));
}

template<int ARGC>
void VM::bind_method(PyObject* obj, Str name, NativeFuncRaw fn) {
    check_type(obj, tp_type);
    obj->attr().set(name, VAR(NativeFunc(fn, ARGC, true)));
}

template<int ARGC>
void VM::bind_func(PyObject* obj, Str name, NativeFuncRaw fn) {
    obj->attr().set(name, VAR(NativeFunc(fn, ARGC, false)));
}

void VM::_error(Exception e){
    if(callstack.empty()){
        e.is_re = false;
        throw e;
    }
    top_frame()->push(VAR(e));
    _raise();
}

PyObject* VM::_exec(){
    Frame* frame = top_frame();
    i64 base_id = frame->id;
    PyObject* ret = nullptr;
    bool need_raise = false;

    while(true){
        if(frame->id < base_id) UNREACHABLE();
        try{
            if(need_raise){ need_raise = false; _raise(); }
            ret = run_frame(frame);
            if(ret == _py_op_yield) return _py_op_yield;
            if(ret != _py_op_call){
                if(frame->id == base_id){      // [ frameBase<- ]
                    callstack.pop();
                    return ret;
                }else{
                    callstack.pop();
                    frame = callstack.top().get();
                    frame->push(ret);
                }
            }else{
                frame = callstack.top().get();  // [ frameBase, newFrame<- ]
            }
        }catch(HandledException& e){
            continue;
        }catch(UnhandledException& e){
            PyObject* obj = frame->pop();
            Exception& _e = CAST(Exception&, obj);
            _e.st_push(frame->snapshot());
            callstack.pop();
            if(callstack.empty()) throw _e;
            frame = callstack.top().get();
            frame->push(obj);
            if(frame->id < base_id) throw ToBeRaisedException();
            need_raise = true;
        }catch(ToBeRaisedException& e){
            need_raise = true;
        }
    }
}

}   // namespace pkpy