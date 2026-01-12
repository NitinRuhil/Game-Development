Project: A1 - Part 2 - 2D Platformer Scene
Name: Nitin Ruhil
Student ID: V01016220
Unity Version: 2022.3.62f2 (Built-in 2D)

---------------------------------------------------------
Requirements Completed
---------------------------------------------------------
1) Two-dimensional scene with static platforms. Layout is traversable. (Req. 1)

2) Two vertical platforms reachable:
   - First wooden platform above ground.
   - Second wooden platform placed higher, reachable from the first. (Req. 2)

3) Two horizontal platforms reachable by jumping:
   - Ground gap between dirt blocks.
   - Jump between elevated wooden platform and dirt/second platform. (Req. 3)

4) Grid, Tilemap, and Tile Palette used to build the scene. (Req. 4)

5) Three distinct tilesheets imported and used:
   - Dirt/Ground tiles
   - Wooden block tiles (platforms)
   - Sky/Background tiles
   (all sourced from free 2D game art sites such as Kenney.nl / OpenGameArt). (Req. 5)

6) All sprites packed into a single Sprite Atlas:
   - Atlas file: Assets/Atlas/MainAtlas.spriteatlas
   - "Include in Build" enabled. (Req. 6)

7) Background and Foreground layers separated:
   - Sky tiles on Sorting Layer: Background
   - Dirt/Wood tiles on Sorting Layer: Midground
   - (Props/decor can be placed in Foreground if needed). (Req. 7)

8) Camera background color changed to match sky. (Req. 8)

9) Quality:
   - Consistent PPU = 64 across all tilesheets.
   - Sprites sliced at 64×64, Filter Mode = Point (no filter), Compression = None.
   - Layout is non-trivial with vertical + horizontal reachability. (Req. 9)

---------------------------------------------------------
Assumed Reachability Rule
---------------------------------------------------------
•⁠  ⁠Jump height = 3 tiles, horizontal jump = 3 tiles.
•⁠  ⁠All vertical gaps are ≤ 3 tiles.
•⁠  ⁠All horizontal gaps are ≤ 3 tiles.

---------------------------------------------------------
Notes
---------------------------------------------------------
•⁠  ⁠Main scene file: Assets/Scenes/Main.unity
•⁠  ⁠Sources of art: https://opengameart.org/content/wood-texture-tiles

https://opengameart.org/content/ground-0

https://opengameart.org/content/sky-backdrop

•⁠  ⁠Excluded auto-generated folders from submission zip: Library/, Logs/, Temp/, Obj/, UserSettings/
