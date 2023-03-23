# Boids

![alt text](https://github.com/lbarazza/Boids/blob/main/media/boids.gif "Flockers gif")

*Simulation of two storms of birds with boids. The two storms are represented by the two different colors. The white birds are scared of the green ones. So, when they come near a white bird, they tend to fly away.*

## Boids Simulation

This boids simulation is based on the [algorithm](http://www.red3d.com/cwr/boids/) developed by Craig Reynolds.
Every bird has to follow three very simple rules:

1. Separation: steer to avoid crowding local flockmates
2. Alignment: steer towards the average heading of local flockmates
3. Cohesion: steer to move toward the average position of local flockmates

The following of these three simple rules results in a large scale emergent flock-like behaviour.

## Further reading

- http://www.red3d.com/cwr/boids/
- http://www.cs.toronto.edu/~dt/siggraph97-course/cwr87/
- https://cs.stanford.edu/people/eroberts/courses/soco/projects/2008-09/modeling-natural-systems/boids.html
