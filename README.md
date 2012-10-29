Torque3D CE
===========

This is a work-in-progress port of the [Torque 3D Community Edition](https://lab.collateral-studios.eu/trac/wiki) project to the MIT Licensed Open Source version of [Torque 3D](http://www.garagegames.com/products/torque-3d) from [GarageGames](http://www.garagegames.com).

* The ceport branch will be the working snapshot of the CE project.  It will be based upon the master branch, with all changes verified as working against the master branch.  
* Topic branches will be used as transitional branches to contain the work-in-progress commits.  Once the work for a specific topic is complete that branch is merged into the ceport branch.  There should be no direct commits to the master or devlopment branch.  
* Binaries and other executables will rarely, if ever, be updated here.  You are responsible for compiling the project on your end.
* The testing project will be bruilt from the Full Template with Bullet and other optional build components enabled.  Look to the Full Template's project.conf for an example of the conditional build options if using any other template project.
* Any Community Edition member who has read & write access to the license restricted repository can ask for read & write access to this GitHub repository as well.  All others will be evaluated.
* Pull requests are welcome!

More Information
---------------- 

* [Torque 3D Community Edition Is Alive](http://www.garagegames.com/community/blogs/view/21727)
* [Torque 3D Community Edition Discussion Thread](http://www.garagegames.com/community/forums/viewthread/130724)
* [Torque 3D's Achilles heel](http://www.garagegames.com/community/forums/viewthread/130118)

Compiling Torque 3D
If you are going to be using a PhysX template, then you will need to complete this step. Otherwise, you can go to 2a.
1. Install the PhysX SDK version 2.8.4.6.
  * In a web browser, go to the NVidia Support Center (http://supportcenteronline.com/ics/support/default.asp?deptID=1949)
  * If you do not have an account, you will need to register with them to have the support staff create an account for you.
  * If you have an account, login.
  * On the middle of the page, on the right, click on Downloads.
  * On the far right column, under Old downloads, click More.
  * Download the Windows 2.8.4.6 version.
  * Run the installer and follow the steps to install it in the default location.
  * Depending on your operating system version, you may need to reboot after the installation.

2a. Creating a New Game Project - With Toolbox
  * Run the Torque 3D Toolbox.
  * Click New Project.
  * Choose a base template.
  * Name the Project.
  * Click Create to create the project.
  * When it finishes, click the Finished button.

2b. Creating a New Game Project - Manually
  * Open the Templates directory.
  * Right-click and copy the template that you want to use.
  * Navigate to the directory where you want to develop your game.
  * Paste inside of that directory to copy the files.
  * Open the game directory.
  * Rename the executable and DLL files to the name of your game (for example, rename Full.exe to MyGame.exe if you used the Full template).
  * Rename the plugin's in the same way, replacing the template name with your game's name.
  * Inside your project directory, double-click the generateProjects.bat file.
  * It will create the solutions.
 
3. Compiling Torque 3D
  * Navigate to engine\build\Visual Studio 2010
  * Double-click the Torque 3D.sln file.
  * When Visual Studio 2010 is fully loaded, press F5 to perform a clean build.

License
-------

Copyright (c) 2012 GarageGames, LLC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
