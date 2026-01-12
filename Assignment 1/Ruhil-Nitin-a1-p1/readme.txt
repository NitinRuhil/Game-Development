Assignment A1 – Part 1 (3D Platformer Scene)
============================================

Student Name: Nitin Ruhil
Project: Ruhil-Nitin-A1-p1
Unity Version: 2022.3 LTS (macOS)
Scene Name: Scene_P1_3D.unity
Theme: Rooftop Parkour Garden
Project: A1 - Part 1 - 3D Platformer Scene=

---------------------------------------------------------
Requirements Completed
---------------------------------------------------------
1) Constructed a static 3D scene with volume using all three dimensions.
   - Platforms positioned at different x, y, and z coordinates. (Req. 1)

2) Two vertical platforms reachable:
   - Lower platform → higher platform.
   - Another platform requiring a drop down from higher to lower. (Req. 2)

3) Two horizontal platforms reachable by jumping:
   - Platforms offset in the x direction within reachable jump distance.
   - Another pair aligned for lateral movement. (Req. 3)

4) Used at least three different Unity 3D object primitives:
   - Cube (platforms)
   - Cylinder (pillar)
   - Plane (ground) (Req. 4)

5) Created and applied at least two custom materials:
   - Material #1 (applied to ground plane)
   - Material #2 (applied to pillar/platforms)
   - No object uses default material. (Req. 5)

6) Used ProBuilder to create additional geometry:
   - Staircase connecting two areas
   - Polygon-shaped floor/structure created with polygon shape tool
   - One additional custom geometry (non-standard primitive) (Req. 6)

7) Added at least two 3D geometry assets from external sources (Kenney.nl, OpenGameArt, Unity Asset Store). (Req. 7)

8) Created a new material using the **Procedural Skybox Shader**:
   - Modified settings: sky tint, exposure, sun size, directional light orientation.
   - Verified skybox is visually different from Unity’s default skybox. (Req. 8)

9) Layout & scaling quality:
   - Objects scaled appropriately, aligned cleanly.
   - Platforms placed logically to create traversable space.
   - Scene layout is non-trivial (not random placement). (Req. 9)

---------------------------------------------------------
Assumed Reachability Rule
---------------------------------------------------------
- Jump height = 3 units, horizontal jump = 3 units.
- All vertical and horizontal distances are set within these limits.

---------------------------------------------------------
Notes
---------------------------------------------------------
- Main scene file: Assets/Scenes/Part1_3D.unity
- Three custom primitives: Cube, Cylinder, Plane
- ProBuilder objects: Staircase, polygonal building, custom shape
- Two external 3D models imported from free asset libraries

https://assetstore.unity.com/packages/3d/props/metal-door-5397
- Custom materials created and applied
- Excluded auto-generated folders (Library/, Logs/, Temp/, Obj/, UserSettings/) from submission zip