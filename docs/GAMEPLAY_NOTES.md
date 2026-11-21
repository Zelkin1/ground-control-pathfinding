# Gameplay notes

This file lists the changes of the simulator that can change more than the movement of units if they go into the game. Each entry says what changes, why it can matter for gameplay, and what to check in the game before the change goes in. PARAMETERS.md lists the values of the simulator, and NOTES.md at the project root holds the research notes.

## Line of sight past parked squads (not verified in the game)

In the formation modes (the form and coord modes), the waypoint following of the simulator treats the straight line from a unit to a waypoint as blocked when the line brings the body of the unit within 0.5 m (`SIGHT_TOUCH_MARGIN_METERS`) of a unit of a parked squad. A parked squad is a squad of any team that neither moves nor searches. The game itself checks only for blocking terrain on that line. The code is `is_in_sight` in simulator/src/game/movement.c.

The simulator uses this test only for movement. It skips to a later waypoint that is in sight, it searches again for a waypoint that is hidden, and it takes the next waypoint on arrival. The simulator has no weapons, so its results say nothing about shooting.

In the game, a line-of-sight test can also decide whether a unit sees a target and can fire at it. If the mod puts the extra test into a function that the target search or the weapons also use, units can stop seeing or firing at targets past an allied squad, or past any parked squad. TODO: confirm which line test the target search (0x507ce0 and 0x507db0, see PATHFINDING.md) and the weapons call.

Before this change goes into the game, do these steps:

1. Find the line test that the target search and the weapons of the game call, and compare it with the one that waypoint following calls.
2. If they share a function, put the extra test into a separate function that only waypoint following calls.
3. In the game, make sure that a unit that fires past a parked allied squad behaves as in the base game.
4. In the game, make sure that waypoint following searches a way around a parked squad that blocks its route, which is the purpose of the change.
