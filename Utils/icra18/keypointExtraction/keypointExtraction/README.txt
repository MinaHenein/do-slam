---=== EDIT CONTOUR ===---

AUTHOR
	Vincent Garcia
	
DESCRIPTION
	The current directory contains functions allowing to extract keypoints 
	also called "feature points", "corners", "interest points", etc.
	The following algorithm are provided :
		- Harris
		- SUSAN
		- Harris-Laplace
		- Laplacian of Gaussian (LoG)
		- Gilles
	The input image must be a gray level image. The output is a matrix of
	dimension Nx2 or Nx3 with N the number of keypoints extracted. The first
	column gives the row poisition of the keypoints and the second column
	gives the column position of the keypoints. The third column gives
	the feature scale of the keypoints. This scale corresponds to the radius
	of the local neighborhood to consider. Note that the SUSAN algorithm is 
	a contribution of Joaquim Luis.
	
	Run the test function to see an example.
	
	The picture "door" was taken by Aleksandra Radonic'.
	The picture "sunflower" was taken by Candy Torres.
	The picture "patrol" was taken by Markus Schöpke.
	
LEGAL
	1. The enclosed function can be freely reused under a Creative Commons
	   License (http://creativecommons.org/licenses/by-nc-sa/2.0) :
	   You are free:
		    * to Share — to copy, distribute and transmit the work
		    * to Remix — to adapt the work
		Under the following conditions:
		    * Attribution. You must attribute the work in the manner specified
			  by the author or licensor (but not in any way that suggests that
			  they endorse you or your use of the work).
		    * Noncommercial. You may not use this work for commercial purposes.
		    * Share Alike. If you alter, transform, or build upon this work, you
			may distribute the resulting work only under the same or similar
			license to this one.
			
	2.	The pictures provided were choosen on flickr.com. These pictures can be
		freely reused under a Creative Commons License. The authors are Aleksandra Radonic',
		Candy Torres, and Markus Schöpke.