# BitFS Final Speed Transfer Brute Forcer
A GPU-based brute forcing tool to search for working setups for the FST step of the BitFS 0xA TAS. This version of the brute forcer searches for slide-kick 10k PU routes. Full solutions are output into a CSV file (see -o in Options for more details).

This tool evolved out of Tyler Kehne's platform max tilt brute forcer, but has since come to encompass many other aspects of the FST setup.

This is a SYCL port of the original CUDA version.

## Options ##
This program accepts the following options:

#### Platform Normal Search Settings ####
<pre>
-nx &lt;min_nx&gt; &lt;max_nx&gt; &lt;n_samples&gt;:          Inclusive range of x normals to be considered, and the number of normals to sample.
                                              If min_nx==max_nx then n_samples will be set to 1.
  
-nxz &lt;min_nxz&gt; &lt;max_nxz&gt; &lt;n_samples&gt;:       Inclusive range of xz sums to be considered, and the number of z normals to sample.
                                              If min_nxz==max_nxz then n_samples will be set to 1.
                                              To search negative z normals, set to min_nxz and max_nxz to negative values.
  
-ny &lt;min_ny&gt; &lt;max_ny&gt; &lt;n_samples&gt;:          Inclusive range of y normals to be considered, and the number of normals to sample.
                                              If min_ny==max_ny then n_samples will be set to 1.

-nz:                                        Search by z normal instead of xz sum.
                                              Ranges supplied with -nxz will be converted to z normal ranges.
  
-q:                                         Search all 8 "quadrants" simultaneously. Overrides platform position set by -p.
</pre>

#### Brute Forcer Settings ####
<pre>
-f &lt;frames&gt;:                                Maximum frames of platform tilt considered.
  
-pu &lt;frames&gt;:                               Number of frames of PU movement for 10k PU route.
                                              Currently, only 3 frame routes are supported.
  
-dx &lt;delta_x&gt;:                              x coordinate spacing of positions on the platform.
  
-dz &lt;delta_z&gt;:                              z coordinate spacing of positions on the platform.
  
-p &lt;platform_x&gt; &lt;platform_y&gt; &lt;platform_z&gt;:  Position of the pyramid platform.
</pre>
#### Output Settings ####
<pre>
-l &lt;path&gt;:                                  Path to the log file.
  
-o &lt;path&gt;:                                  Path to the output file.

-c &lt;path&gt;:                                  Path to the checkpoint file.

-m:                                         Output mode. The amount of detail provided in the output file.
                                              0: Minimal output. Prints all normals with full solutions, along with number of full solutions found.
                                              1: Minimal output with partial solutions. Prints all normals with 10k partial solutions or better, along with the latest stage with solutions.
                                              2: Full output. Prints all normals with full solutions, along with full details of the setup.
</pre>
#### GPU Settings ####
<pre>
-d &lt;device_id&gt;:                             The SYCL device used to run the program.
  
-t &lt;threads&gt;:                               Size of SYCL work group to assign to the program.
  
-lsk1 &lt;n_solutions&gt;:                        Maximum number of phase 1 solutions for 10k setup search.
  
-lsk2a &lt;n_solutions&gt;:                       Maximum number of phase 2a solutions for 10k setup search.
  
-lsk2b &lt;n_solutions&gt;:                       Maximum number of phase 2b solutions for 10k setup search.
  
-lsk2c &lt;n_solutions&gt;:                       Maximum number of phase 2c solutions for 10k setup search.
  
-lsk2d &lt;n_solutions&gt;:                       Maximum number of phase 2d solutions for 10k setup search.
  
-lsk3 &lt;n_solutions&gt;:                        Maximum number of phase 3 solutions for 10k setup search.
  
-lsk4 &lt;n_solutions&gt;:                        Maximum number of phase 4 solutions for 10k setup search.
  
-lsk5 &lt;n_solutions&gt;:                        Maximum number of phase 5 solutions for 10k setup search.
  
-lsk6 &lt;n_solutions&gt;:                        Maximum number of phase 6 solutions for 10k setup search.
  
-lp &lt;n_solutions&gt;:                          Maximum number of platform tilt solutions.
  
-lu &lt;n_solutions&gt;:                          Maximum number of upwarp solutions.
  
-lsku &lt;n_solutions&gt;:                        Maximum number of slide kick upwarp solutions.
  
-ls &lt;n_solutions&gt;:                          Maximum number of speed solutions.
  
-l10k &lt;n_solutions&gt;:                        Maximum number of 10k solutions.
  
-lsl &lt;n_solutions&gt;:                         Maximum number of slide solutions.
  
-lbd &lt;n_solutions&gt;:                         Maximum number of breakdance solutions.
  
-ld10k &lt;n_solutions&gt;:                       Maximum number of double 10k solutions.
  
-lbp &lt;n_solutions&gt;:                         Maximum number of bully push solutions.
  
-lsq &lt;n_squish_spots&gt;:                      Maximum number of squish spots.
  
-lst &lt;n_strain_setups&gt;:                     Maximum number of strain setups.
</pre>
#### Misc Settings ####
<pre>
-b:                                         Disable buffering on stdout and stderr.

-v:                                         Verbose mode. Prints all parameters used in the brute forcer.

-s:                                         Silent mode. Suppresses all print statements output by the brute forcer.
  
-h --help:                                  Prints the help text.
</pre>

## Dependencies ##
To maximise throughput, this program has been written in SYCL to allow for processing on the GPU. Therefore, to build this program you will need to install the Intel oneAPI BaseKit.  

In addition, to run the program you will need a computer with a SYCL compatible GPU. For AMD GPUs you will also need to install the oneAPI for AMD GPUs plugin (currently only available on Linux systems).

## Building Instructions ##
This program can be built with Visual Studio or CMake using the included config files. 

For Visual Studio builds, you might need to link CUDA Build Customizations to the project before building. Instructions for doing that can be found here:

https://docs.nvidia.com/cuda/cuda-installation-guide-microsoft-windows/index.html#compiling-cuda-programs

For CMake builds, use the following commands:

```
cmake .
make
```

## Including this in your own projects ##
The brute forcer includes a basic API to run the program from external projects. The code snippet below shows an example of how this works:

```
#include <fstream>
#include <iostream>
#include <cstring>
#include <string>
#include "FST.hpp"
  
std::string outFile = "logFile.log"; // Path to log file
std::string outFile = "outData.csv"; // Path to solution csv file
std::ofstream wf; // Output stream for solution csv file
std::ofstream logf; // Output stream for log file
struct FSTData p; // Pointers to structures used by brute forcer
struct FSTOptions o; // Options for the brute forcer

// Open log file
// You can skip this if you don't want to output to a log file
logf.open(s.logFile);

// Set any options you want to change
o.nThreads = 128;

// Allocate memory and initialise the brute forcer structures
int error = initialise_fst_vars(&p, &o, logf);

// You may get errors if you don't have enough memory
// Check output variable for no errors
if (error == 0) {
    // Set up output csv file
    // You can skip this if you don't want to output them to a file
    initialise_solution_file_stream(wf, outFile, &o);

    // Pick a normal (or normals) you want to test
    float testNormal[3] = {0.1808f, 0.87768f, -0.396f};
  
    // Check if the normal has any solutions
    if (check_normal(testNormal, &o, &p, wf, logf) == STAGE_COMPLETE) {
          // Do stuff with successful normals
    }

    // When you're done, release the memory assigned to the brute forcer
    free_fst_vars(&p);
    wf.close();
    logf.close();
} else {
      // Report errors
}
```

## Credits ##
**SpudY2K** - Current brute forcer implementation.  
**Tyler Kehne** - Original max tilt brute forcer.  
**AnthonyC4** - Original upwarp brute forcer. Implementations of platform, surface, trig, and math functions.  
**Modiseus** - Boundary distance logic.  

Additional thanks to **Dan Park**, **Diffractor**,  **dtracers**, **TheLonelyPoire**, and **superminer** for their help in optimisation and bug fixing.  

And a final special thanks to all contributors on the SM64 decompilation project. Without their excellent work this entire project would be impossible.
