<!--
Maintainer:   jeffskinnerbox@yahoo.com / www.jeffskinnerbox.me
Version:      0.0.0
-->


<div align="center">
<img src="https://raw.githubusercontent.com/jeffskinnerbox/blog/main/content/images/banners-bkgrds/work-in-progress.jpg"
        title="These materials require additional work and are not ready for general use." align="center" width=420px height=219px>
</div>


---------------

# Mini Loki - Omnidirectional Robotic Platform

* [Mini Loki - Omnidirectional robotic platform](https://cults3d.com/en/3d-model/gadget/mini-loki-omnidirectional-robotic-platform)
* [Mini Loki - Omnidirectional Robotic Platform](https://www.myminifactory.com/object/3d-print-mini-loki-omnidirectional-robotic-platform-17786)
  * [Github Repository: davidsanfal / miniloki](https://github.com/davidsanfal/miniloki)

---------------


* rpi-camera.scad - <https://github.com/larsch/openscad-modules/blob/master/rpi-camera.scad>
* misc_parts.scad - <https://github.com/RigacciOrg/openscad-rpi-library/blob/master/misc_parts.scad>
* misc_boards.scad - <https://github.com/RigacciOrg/openscad-rpi-library/blob/master/misc_boards.scad>
* N20-DC-Motor.stl - <https://www.printables.com/model/735391-n20-dc-motor/files>
* N20-DC-Motor-with-Encoder.stl - <https://grabcad.com/library/n20-motor-with-magnetic-encoder-1>

# Sites With 3D Print Plans

Here are some Internet sties that have posted STL files we many want to use or modify to our needs.
Most are free to download and the others typically cost only $1 to $5 to download.
All you need is to put the STL file thought a slicer and then print it.

* [Thingiverse](https://www.thingiverse.com/)
* [Maker World](https://makerworld.com/en)
* [Thangs](https://thangs.com/?sort=trending)
* [MyMiniFactory](https://www.myminifactory.com/)
* [Cults](https://cults3d.com/)
* [Yeggi](https://www.yeggi.com/)
* [Printables](https://www.printables.com/)

# Examples

* <https://www.thingiverse.com/search?q=N20+motor+mount+housing&page=1>
  * [N20 gear motor mount w/nut trap](https://www.thingiverse.com/thing:878347)
  * [N20 Motor Mounting Bracket (Improved)](https://www.thingiverse.com/thing:6895505)
  * [N20 Motor Holder](https://www.thingiverse.com/thing:2431632)

* <https://www.thingiverse.com/search?q=Omniwheel+Robot&page=1>
  * [Mini Loki - Omnidirectional robotic platform](https://www.thingiverse.com/thing:1276446)


---------------


# Alternative Physical Design

* [TurtleBot3 Friends: Omni](https://github.com/ROBOTIS-GIT/emanual/tree/master/docs/en/platform/turtlebot3#turtlebot3-friends-omni)
* [ROBOTIS TurtleBot Omni](https://www.thingiverse.com/thing:3069809#google_vignette)


---------------


# Problematic STL Files

When importing a STL (Standard Triangle Language) file from some 3D printing website
into OpenSCAD (e.g `openscad-object = import("my-file.stl")`),
I have on several occasions had OpenSCAD make complain about the integrity of the STL file.
The process of fixing the STL file is not automatic and you may need to try several things blindly.
Consider asking Gemini "_Is there any tool that will give automatic mesh optimization?_"
and it will give you some free tools you can use.

The problem typically stems from the mesh of the STL file not being [manifold][03]
(a mathematical way of saying it's not a perfectly sealed, continuous surface).
OpenSCAD requires a clean, manifold mesh to perform its operations correctly.
Here are the most frequent culprits:

* **Non-Manifold Edges:** An edge of a triangle is shared by more than two other triangles.
  Think of it like a "T" junction in a wall – it creates ambiguity about what's "inside" and "outside."
* **Holes:** The mesh has gaps, meaning it's not a "watertight" solid. This is a very common issue.
* **Self-Intersections:** Triangles in the mesh pass through each other. This can confuse OpenSCAD's geometry engine.
* **Flipped Normals:** The "normal" of a triangle indicates its front or back face.
  If some normals are pointing inward while others point outward, it creates an incoherent surface.
* **Degenerate Faces:** These are triangles with zero area, essentially a line or a point, which can cause calculation errors.

A STL file that won't import successfully into OpenSCAD appears to be a common frustration,
but often fixable via [MeshLab][01] and [FreeCAD][].

## Step-by-Step Cleaning Process Using MeshLab

Here’s a MeshLab typical workflow:

1. Open your STL file in MeshLab.
2. Go to Filters -> Cleaning and Repairing -> Select Non-Manifold Edges. If any edges are highlighted, you have non-manifold issues.
3. To automatically attempt a fix, go to Filters -> Cleaning and Repairing -> Repair Non-Manifold Edges.
4. Next, look for holes. Go to Filters -> Cleaning and Repairing -> Close Holes. You may need to adjust the "Max hole size" parameter.
5. It's also a good practice to unify the normals. Go to Filters -> Normals, Curvatures and Orientation -> Re-Orient All Faces Coherently.
6. Finally, export your cleaned mesh as a new STL file (File -> Export Mesh As...).
7. If you're still having trouble, you can try some of MeshLab's more advanced filters, but the steps above will solve the majority of import problems.

## Step-by-Step Cleaning Process Using FreeCAD

FreeCAD can also be helpful with problems.
OpenSCAD can import both ASCII and binary STL file formats,
and FreeCAD can be used for converting a binary STL file to the ASCII format,
and eliminate some problems.

Here’s a typical FreeCAD workflow:

1. Open your binary STL file in FreeCAD.
2. Select the mesh object in the Model tree.
3. Go to File > Export....
4. Select STL Mesh (*.stl) as the file type.
5. Save the file. FreeCAD typically saves STLs in ASCII format by default when exporting from the File menu.



[01]:https://www.meshlab.net/
[03]:https://www.quora.com/What-is-a-manifold-in-laymans-terms

