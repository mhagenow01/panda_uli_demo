#include <stdio.h>
#include <omp.h>

extern "C" {
    long sumTricker(long i){
        return i+2;
    }

    void loopSerial()
    {
        long N = 5000000000;
        for (long i=0; i<N; i++) {
            long temp = sumTricker(i);
        }
    }

    void loopParallel()
    {
        long N = 5000000000;

        omp_set_num_threads(8);        
        #pragma omp parallel for
        for (long i=0; i<N; i++) {
            long temp = sumTricker(i);
        }
    }
}
