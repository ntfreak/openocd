-- Duane Ellis'es test case for building numerous openocd configurations...
Dec 26,2008
---------------------------------------------------------------------------

1)	Make a directory some where..

	mkdir ~/test

2)	Change to that directory

	cd  ~/test

3)	Checkout OpenOCD in that directory.

	cd ~/test
	svn co https://svn.berlios.de/svnroot/repos/openocd/trunk openocd

4)      Copy the "build.test1" directory to the "~/work" directory.


	cd ~/test
	cp ~/openocd/testing/build.test1/.  ~/test/.

5)	If needed, download various components.

	cd ~/work
	make all.download


6)	For Linux - type: 

	cd ~/work
	make linux.buildtest

7)	For Cygwin - type:

	cd ~/work
	make cygwin.buildtest
