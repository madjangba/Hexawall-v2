sudo g++ engdemo.cpp -o engdemo2 -I/usr/local/MATLAB/R2014a/extern/include/cpp -I/usr/local/MATLAB/R2014a/extern/include -DGLNXA64 -DGCC -O -DNDEBUG -Wl,-rpath-link,/usr/local/MATLAB/R2014a/bin/glnxa64 -L/usr/local/MATLAB/R2014a/bin/glnxa64 -leng -lmx -lm


-ansi -D_GNU_SOURCE -I/usr/local/MATLAB/R2014a/extern/include/cpp -I/usr/local/MATLAB/R2014a/extern/include -DGLNXA64 -DGCC

-O -DNDEBUG

-Wl,-rpath-link,/usr/local/MATLAB/R2014a/bin/glnxa64 -L/usr/local/MATLAB/R2014a/bin/glnxa64 -leng -lmx -lm
