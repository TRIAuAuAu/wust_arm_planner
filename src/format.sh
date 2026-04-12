find . -path ./build -prune -o \
        -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.c' -o -name '*.cu' -o -name '*.cpp' \) \
        -exec clang-format -i {} +
