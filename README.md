# Geodesic Patterns

This repo contains my implementation of several methods for generating geodesic curve patterns on an architectural context; and where developed for my _Master Thesis_ on _Parametric Design in Architecture_. A link to my master thesis will also be published once presented.

This method's were originally introduced by Helmut Pottman in [_Geodesic Patterns_](http://www.geometrie.tugraz.at/wallner/geopattern.pdf).

All methods  where implemented as components for the Rhino + Grasshopper3D software and have been tested on the latest version (Rhino 6 on Windows) and (Rhino 5 WIP on Mac). Components should work cross-plattform.

> Please read the dependencies section below.

## To-do's

- Level-set method
- Geodesic vector fields

## Dependencies

- Rhino6+Grasshopper3D

> THIS PROJECT WAS DEVELOPED ON A MAC... therefore, you will not be able to compile it on Windows directly. If you are using Windows, you should erase the references that throw an error and replace them with the appropriate Windows references (RhinoCommon.dll, Grasshopper.dll, GH_IO.dll).

- [CSNumerics - Portable numerical algorithms in C#](https://github.com/cureos/csnumerics)

> You should clone and build your own CSNumerics.dll because the latest version on GitHub is **NOT** the latest one (which is the one I'm using)

## Other info

Pending...

