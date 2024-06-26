python prebuild.py

SRC_C=$(find src/ -name "*.c")
SRC_CPP=$(find src/ -name "*.cpp")
SRC="$SRC_C $SRC_CPP"

clang++ -std=c++17 --coverage -O1 -stdlib=libc++ -frtti -Wfatal-errors -o main src2/main.cpp $SRC -Iinclude -DPK_ENABLE_OS=1 -DPK_DEBUG_PRECOMPILED_EXEC=1 -DPK_ENABLE_PROFILER=1

python scripts/run_tests.py

# if prev error exit
if [ $? -ne 0 ]; then
    exit 1
fi

rm -rf .coverage
mkdir .coverage
rm pocketpy_c.gcno

UNITS=$(find ./ -name "*.gcno")
llvm-cov-15 gcov ${UNITS} -r -s include/ -r -s src/ >> .coverage/coverage.txt

mv *.gcov .coverage
rm *.gcda
rm *.gcno
