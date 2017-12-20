% Edge OR Corners detecting OR image smoothing using SUSAN technique.
%
% There are three different types of output:
%   1. Logical (mask) array - This is the deffault type used with the edge detection
%   2. Uint8 array - When '-ei' or '-ci' options are used
%   3. Mx2 array of doubles with the indice corner coordinates
%
% Usage:
%   OUT = susan(GRAY, [options]);
%
%   Where GRAY is a MxN intensity image
%   and [options] is any of the following '-opt' strings:
% 
% -e : Edges mode - output a mask (logical) edges image [default]
% -ei: Edges Image mode (overlay edges on input image)
% -s : Smoothing mode
% -c : Corners mode - output a Mx2 corners matrix with (image x,y corners coordinates)
% 
% -ci: Corners Image mode (overlay corners on input image)
% 
% See source code for more information about setting the thresholds
% -t <thresh> : Brightness threshold, all modes (default=20)
% -d <thresh> : Distance threshold, smoothing mode, (default=4) (use next option instead for flat 3x3 mask)
% -3 : Use flat 3x3 mask, edges or smoothing mode
% -n : No post-processing on the binary edge map (runs much faster); edges mode
% -q : Use faster (and usually stabler) corner mode; edge-like corner suppression not carried out; corners mode
% -b : Mark corners/edges with single black points instead of black with white border; corners or edges mode
% -p : Output initial enhancement image only; corners or edges mode (default is edges mode)
% SUSAN Version 2l (C) 1995-1997 Stephen Smith, DRA UK. steve@fmrib.ox.ac.uk
% 
% Example1:  Show original image overlain with detected edges
%   if IMG is a MxN image
%   edges = susan(IMG,'-ei');
%   image(edges)
%
% Example2:  Show edges only
%   edges = susan(IMG,'-e');
%   image(edges)        % imshow works better
%
% Example3:  Show original image overlain with detected corners
%   edges = susan(IMG,'-ci');
%   image(edges)

%
% Author (of the mexification (not so minor work))
%       Joaquim Luis, jluis@ualg.pt
%
% Original C code and algorithm description at:
%   http://www.fmrib.ox.ac.uk/~steve/susan/
