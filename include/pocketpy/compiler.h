#pragma once

#include "codeobject.h"
#include "common.h"
#include "expr.h"
#include "obj.h"
#include <array>

namespace pkpy{

class Compiler;
typedef void (Compiler::*PrattCallback)();

struct PrattRule{
    PrattCallback prefix;
    PrattCallback infix;
    Precedence precedence;
};

class Compiler {
    PK_ALWAYS_PASS_BY_POINTER(Compiler)
    friend constexpr std::array<PrattRule, kTokenCount> _init_pratt_rules();

    Lexer lexer;
    stack_no_copy<CodeEmitContext> contexts;
    VM* vm;
    bool unknown_global_scope;     // for eval/exec() call
    // for parsing token stream
    int i = 0;
    std::vector<Token> tokens;

    const Token& prev() const{ return tokens.at(i-1); }
    const Token& curr() const{ return tokens.at(i); }
    const Token& next() const{ return tokens.at(i+1); }
    const Token& err() const{
        if(i >= tokens.size()) return prev();
        return curr();
    }
    void advance(int delta=1) { i += delta; }

    CodeEmitContext* ctx() { return &contexts.top(); }
    CompileMode mode() const{ return lexer.src->mode; }
    NameScope name_scope() const;
    CodeObject_ push_global_context();
    FuncDecl_ push_f_context(Str name);
    void pop_context();

    bool match(TokenIndex expected);
    void consume(TokenIndex expected);
    bool match_newlines_repl();

    bool match_newlines(bool repl_throw=false);
    bool match_end_stmt();
    void consume_end_stmt();

    /*************************************************/
    void EXPR();
    void EXPR_TUPLE(bool allow_slice=false);
    Expr_ EXPR_VARS();  // special case for `for loop` and `comp`

    template <typename T, typename... Args>
    unique_ptr_128<T> make_expr(Args&&... args) {
        void* p = pool128_alloc(sizeof(T));
        unique_ptr_128<T> expr(new (p) T(std::forward<Args>(args)...));
        expr->line = prev().line;
        return expr;
    }

    void consume_comp(unique_ptr_128<CompExpr> ce, Expr_ expr);

    void exprLiteral();
    void exprLong();
    void exprImag();
    void exprBytes();
    void exprFString();
    void exprLambda();
    void exprOr();
    void exprAnd();
    void exprTernary();
    void exprBinaryOp();
    void exprNot();
    void exprUnaryOp();
    void exprGroup();
    void exprList();
    void exprMap();
    void exprCall();
    void exprName();
    void exprAttrib();
    void exprSlice0();
    void exprSlice1();
    void exprSubscr();
    void exprLiteral0();

    void compile_block_body(void (Compiler::*callback)()=nullptr);
    void compile_normal_import();
    void compile_from_import();
    bool is_expression(bool allow_slice=false);
    void parse_expression(int precedence, bool allow_slice=false);
    void compile_if_stmt();
    void compile_while_loop();
    void compile_for_loop();
    void compile_try_except();
    void compile_decorated();

    bool try_compile_assignment();
    void compile_stmt();
    void consume_type_hints();
    void _add_decorators(const Expr_vector& decorators);
    void compile_class(const Expr_vector& decorators={});
    void _compile_f_args(FuncDecl_ decl, bool enable_type_hints);
    void compile_function(const Expr_vector& decorators={});

    PyVar to_object(const TokenValue& value);
    PyVar read_literal();

    void SyntaxError(Str msg){ lexer.throw_err("SyntaxError", msg, err().line, err().start); }
    void SyntaxError(){ lexer.throw_err("SyntaxError", "invalid syntax", err().line, err().start); }
    void IndentationError(Str msg){ lexer.throw_err("IndentationError", msg, err().line, err().start); }

public:
    Compiler(VM* vm, std::string_view source, const Str& filename, CompileMode mode, bool unknown_global_scope=false);
    Str precompile();
    void from_precompiled(const char* source);
    CodeObject_ compile();
};

struct TokenDeserializer{
    const char* curr;
    const char* source;

    TokenDeserializer(const char* source): curr(source), source(source) {}
    char read_char(){ return *curr++; }
    bool match_char(char c){ if(*curr == c) { curr++; return true; } return false; }
    
    std::string_view read_string(char c);
    Str read_string_from_hex(char c);
    int read_count();
    i64 read_uint(char c);
    f64 read_float(char c);
};


constexpr std::array<PrattRule, kTokenCount> _init_pratt_rules(){
    std::array<PrattRule, kTokenCount> rules{};

// http://journal.stuffwithstuff.com/2011/03/19/pratt-parsers-expression-parsing-made-easy/
#define PK_METHOD(name) &Compiler::name
#define PK_NO_INFIX nullptr, PREC_LOWEST
    for(TokenIndex i=0; i<kTokenCount; i++) rules[i] = { nullptr, PK_NO_INFIX };
    rules[TK(".")] =        { nullptr,                  PK_METHOD(exprAttrib),         PREC_PRIMARY };
    rules[TK("(")] =        { PK_METHOD(exprGroup),     PK_METHOD(exprCall),           PREC_PRIMARY };
    rules[TK("[")] =        { PK_METHOD(exprList),      PK_METHOD(exprSubscr),         PREC_PRIMARY };
    rules[TK("{")] =        { PK_METHOD(exprMap),       PK_NO_INFIX };
    rules[TK("%")] =        { nullptr,                  PK_METHOD(exprBinaryOp),       PREC_FACTOR };
    rules[TK("+")] =        { nullptr,                  PK_METHOD(exprBinaryOp),       PREC_TERM };
    rules[TK("-")] =        { PK_METHOD(exprUnaryOp),   PK_METHOD(exprBinaryOp),       PREC_TERM };
    rules[TK("*")] =        { PK_METHOD(exprUnaryOp),   PK_METHOD(exprBinaryOp),       PREC_FACTOR };
    rules[TK("~")] =        { PK_METHOD(exprUnaryOp),   nullptr,                    PREC_UNARY };
    rules[TK("/")] =        { nullptr,                  PK_METHOD(exprBinaryOp),       PREC_FACTOR };
    rules[TK("//")] =       { nullptr,                  PK_METHOD(exprBinaryOp),       PREC_FACTOR };
    rules[TK("**")] =       { PK_METHOD(exprUnaryOp),   PK_METHOD(exprBinaryOp),       PREC_EXPONENT };
    rules[TK(">")] =        { nullptr,               PK_METHOD(exprBinaryOp),       PREC_COMPARISION };
    rules[TK("<")] =        { nullptr,               PK_METHOD(exprBinaryOp),       PREC_COMPARISION };
    rules[TK("==")] =       { nullptr,               PK_METHOD(exprBinaryOp),       PREC_COMPARISION };
    rules[TK("!=")] =       { nullptr,               PK_METHOD(exprBinaryOp),       PREC_COMPARISION };
    rules[TK(">=")] =       { nullptr,               PK_METHOD(exprBinaryOp),       PREC_COMPARISION };
    rules[TK("<=")] =       { nullptr,               PK_METHOD(exprBinaryOp),       PREC_COMPARISION };
    rules[TK("in")] =       { nullptr,               PK_METHOD(exprBinaryOp),       PREC_COMPARISION };
    rules[TK("is")] =       { nullptr,               PK_METHOD(exprBinaryOp),       PREC_COMPARISION };
    rules[TK("<<")] =       { nullptr,               PK_METHOD(exprBinaryOp),       PREC_BITWISE_SHIFT };
    rules[TK(">>")] =       { nullptr,               PK_METHOD(exprBinaryOp),       PREC_BITWISE_SHIFT };
    rules[TK("&")] =        { nullptr,               PK_METHOD(exprBinaryOp),       PREC_BITWISE_AND };
    rules[TK("|")] =        { nullptr,               PK_METHOD(exprBinaryOp),       PREC_BITWISE_OR };
    rules[TK("^")] =        { nullptr,               PK_METHOD(exprBinaryOp),       PREC_BITWISE_XOR };
    rules[TK("@")] =        { nullptr,               PK_METHOD(exprBinaryOp),       PREC_FACTOR };
    rules[TK("if")] =       { nullptr,               PK_METHOD(exprTernary),        PREC_TERNARY };
    rules[TK("not in")] =   { nullptr,               PK_METHOD(exprBinaryOp),       PREC_COMPARISION };
    rules[TK("is not")] =   { nullptr,               PK_METHOD(exprBinaryOp),       PREC_COMPARISION };
    rules[TK("and") ] =     { nullptr,               PK_METHOD(exprAnd),            PREC_LOGICAL_AND };
    rules[TK("or")] =       { nullptr,               PK_METHOD(exprOr),             PREC_LOGICAL_OR };
    rules[TK("not")] =      { PK_METHOD(exprNot),       nullptr,                    PREC_LOGICAL_NOT };
    rules[TK("True")] =     { PK_METHOD(exprLiteral0),  PK_NO_INFIX };
    rules[TK("False")] =    { PK_METHOD(exprLiteral0),  PK_NO_INFIX };
    rules[TK("None")] =     { PK_METHOD(exprLiteral0),  PK_NO_INFIX };
    rules[TK("...")] =      { PK_METHOD(exprLiteral0),  PK_NO_INFIX };
    rules[TK("lambda")] =   { PK_METHOD(exprLambda),    PK_NO_INFIX };
    rules[TK("@id")] =      { PK_METHOD(exprName),      PK_NO_INFIX };
    rules[TK("@num")] =     { PK_METHOD(exprLiteral),   PK_NO_INFIX };
    rules[TK("@str")] =     { PK_METHOD(exprLiteral),   PK_NO_INFIX };
    rules[TK("@fstr")] =    { PK_METHOD(exprFString),   PK_NO_INFIX };
    rules[TK("@long")] =    { PK_METHOD(exprLong),      PK_NO_INFIX };
    rules[TK("@imag")] =    { PK_METHOD(exprImag),      PK_NO_INFIX };
    rules[TK("@bytes")] =   { PK_METHOD(exprBytes),     PK_NO_INFIX };
    rules[TK(":")] =        { PK_METHOD(exprSlice0),    PK_METHOD(exprSlice1),      PREC_PRIMARY };
    
#undef PK_METHOD
#undef PK_NO_INFIX
    return rules;
}

constexpr std::array<PrattRule, kTokenCount> kPrattRules = _init_pratt_rules();

} // namespace pkpy