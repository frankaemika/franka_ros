for f in $(find -name '*.h' -or -name '*.cpp');
  do clang-format -style=file $f -i;
     echo $f
done
