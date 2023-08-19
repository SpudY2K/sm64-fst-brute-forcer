# BitFS Final Speed Transfer Brute Forcer
A GPU-based brute forcing tool to search for working setups for the FST step of the BitFS 0xA TAS. This version of the brute forcer searches for HAU aligned 10k PU routes. Full solutions are output into a CSV file (see -o in Options for more details).

This tool evolved out of Tyler Kehne's platform max tilt brute forcer, but has since come to encompass many other aspects of the FST setup.

## Options ##
This program accepts the following options:
<pre>
-f &lt;frames&gt;:                                Maximum frames of platform tilt considered.
  
-pu &lt;frames&gt;:                               Number of frames of PU movement for 10k glitch. Only 3 frames are currently supported.

-q1 &lt;min_q1&gt; &lt;max_q1&gt;:                      Range of q-frames to test for frame 1 of 10k PU route.

-q2 &lt;min_q2&gt; &lt;max_q2&gt;:                      Range of q-frames to test for frame 2 of 10k PU route.

-q3 &lt;min_q3&gt; &lt;max_q3&gt;:                      Range of q-frames to test for frame 3 of 10k PU route

-nx &lt;min_nx&gt; &lt;max_nx&gt; &lt;n_samples&gt;:          Inclusive range of x normals to be considered, and the number of normals to sample.
                                            If min_nx==max_nx then n_samples will be set to 1.
  
-nxz &lt;min_nxz&gt; &lt;max_nxz&gt; &lt;n_samples&gt;:       Inclusive range of xz sums to be considered, and the number of z normals to sample.
                                            If min_nxz==max_nxz then n_samples will be set to 1.
  
-ny &lt;min_ny&gt; &lt;max_ny&gt; &lt;n_samples&gt;:          Inclusive range of y normals to be considered, and the number of normals to sample.
                                            If min_ny==max_ny then n_samples will be set to 1.
  
-dx &lt;delta_x&gt;:                              x coordinate spacing of positions on the platform.
  
-dz &lt;delta_z&gt;:                              z coordinate spacing of positions on the platform.
  
-p &lt;platform_x&gt; &lt;platform_y&gt; &lt;platform_z&gt;:  Position of the pyramid platform.
  
-o:                                         Path to the output file.
  
-t &lt;threads&gt;:                               Number of CUDA threads to assign to the program.
  
-v:                                         Verbose mode. Prints all parameters used in brute force.
  
-h --help:                                  Prints this text.
</pre>

## Dependencies ##
To maximise throughput, this program has been written in CUDA to allow for processing on the GPU. Therefore, to build this program you will need to install CUDA Toolkit (v11 or later is recommended).  

In addition, to run the program you will need a computer with CUDA compatible GPU. A GPU with compute capability 5.2 or higher and at least 4GB of RAM is recommended. Lower powered GPUs may still be able to run this program, but some tweaking of the build configuration may be necessary.

## Building Instructions ##
This program can be built with Visual Studio or CMake using the included config files. For CMake builds, use the following commands:

<pre>
cmake .
make
</pre>

## Credits ##
SpudY2K - Current brute forcer implementation.  
Tyler Kehne - Original max tilt brute forcer.  
AnthonyC4 - Original upwarp brute forcer. Implementations of platform, surface, trig, and math functions.  
Modiseus - Boundary distance logic.  

Additional thanks to Dan Park, Diffractor,  dtracers, TheLonelyPoire, and superminer for their help in optimisation and bug fixing.  

And a final special thanks to all contributors on the SM64 decompilation project. Without their excellent work this entire project would be impossible.
