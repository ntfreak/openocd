
This makefile is how I Duane Ellis (openocd@duaneellis.com) builds
openocd test purposes on Cygwin. I have included it here so others
might also make use of the same configuration that I use to develop
Openocd.

--Duane Ellis

To make use of it do the following:

(1)	Check out openocd in the standard way.

For example - in cygwin, type this:

    bash$  mkdir -p /home/duane/test
    bash$  cd /home/duane/test
    bash$  svn co https://svn.berlios.de/svnroot/repos/openocd/trunk openocd

(2)	COPY this folder "right above" where you have OpenOCD.

    bash$  cd /home/duane/test
    bash$  cp ./openocd/testing/build.test2/*   /home/duane/test/.

(3) OPTIONALLY

    You might want to review the file "local.uses"
    Change options and so forth at the top of the file.

(4) 	Initially, you need to download some additional files.
	These include "libftdi", "libconfuse", and the ftd2xx drivers.

(5)	You also need to build the supporting libraries and install them
	(They are installed "locally" only)

	Type this command:

    bash$ cd /home/duane/test

    bash$ make initial.build

    	  which:  (1) downloads files
	  	  (2) builds the libs
		  (3) builds OpenOCD

(6)     As you hack upon OpenOCD... to rebuild OpenOCD...
	
    bash$ cd /home/duane/test

    bash$ make remake

    	  which: (1) re-bootstraps 
	  	 (2) re-configures
		 (3) re-builds
		 (4) re-installs.
   
=======
**END**
=======
