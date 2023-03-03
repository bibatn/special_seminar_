#bsub -n 1 -W 00:10 -o result -e /dev/null OMP_NUM_THREADS=1 ./surface 1.pcd 0.09
mpisubmit.pl -p 1 build/surface 1.pcd 0.09
