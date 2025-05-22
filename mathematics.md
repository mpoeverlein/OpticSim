# Mathematics
## Collision between sphere and line
The line is given by $$\mathbf{r}(t) = \mathbf{o} + t * \mathbf{d}$$. 
By projecting the direction vector $\mathbf{d}$ onto the distance vector between $\mathbf{o}$ and the center of the sphere $\mathbf{c}$, the time $t_p$ can be found. For this time, the line is the closest to the sphere's center.
$$t_p = (\mathbf{c}-\mathbf{o}) \cdot \mathb{d}$$.
This yields $\mathbf{r}_p = \mathbf{o} + t_p * \mathbf{d}$. 
We can compute the distance $D$ between $\mathbf{r}_p$ and $\mathbf{c}$ to compare it with the sphere's radius $R$.
$$D^2 = |\mathbf{r}_p - \mathbf{c}|^2$$
There are the following cases:
  - $D^2 < R^2$: Two collision points
  - $D^2 > R^2$: No collision (miss)
  - $D^2 = R^2$: Exactly one collision (tangent)