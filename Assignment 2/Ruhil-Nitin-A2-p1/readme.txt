CSC 486A – Assignment 2 – Part 1
Physical World and Character Control
Name: Nitin Ruhil
Student ID: V01016220

What I Completed
1. Player Animations (Walk + Idle)
I added a player character with a sprite sheet.
The Idle and Walk animations work correctly.
The animation speed changes based on how fast the player moves.
Transitions are smooth and switch at the right time.

2. Player Movement Script
The player can move left and right without sticking to walls.
Jumping works only when the player is on the ground.
I detect ground contact using an OverlapCircle under the player.
The movement feels smooth and responsive.

3. Player Collider Shape
I set the collider size to match the character sprite.
The collider looks clean and fits the body shape properly.

4. Physical World + Tilemap Collider
My world uses Tilemap Collider and a Composite Collider.
This makes the platforms smooth so the player does not get stuck on edges.

5. Moving Platforms
I added one horizontal moving platform and one vertical moving platform.
Each platform moves between two points and loops.
The player moves correctly with the platforms because I use physics callbacks
(OnCollisionEnter/Exit and re-parenting the player).

6. Camera Follow
I made the camera a child of the player so it follows smoothly.

7. Quality + World Design
I expanded my level so the player can travel to different areas.
There are places that require jumping or using moving platforms.
The world feels like a small playable demo with proper flow.

What I Did Not Complete
- Everything above is completed.
