# Mathematics
## Collision between sphere and line
The line is given by $$\mathbf{r}(t) = \mathbf{o} + t  \mathbf{d}$$. 
By projecting the direction vector $\mathbf{d}$ onto the distance vector between $\mathbf{o}$ and the center of the sphere $\mathbf{c}$, the time $t_p$ can be found. For this time, the line is the closest to the sphere's center.
$$t_p = (\mathbf{c}-\mathbf{o}) \cdot \mathbf{d}$$.
This yields $\mathbf{r}_p = \mathbf{o} + t_p  \mathbf{d}$. 
We can compute the distance $D$ between $\mathbf{r}_p$ and $\mathbf{c}$ to compare it with the sphere's radius $R$.
$$D^2 = |\mathbf{r}_p - \mathbf{c}|^2$$
There are the following cases:
  - $D^2 < R^2$: Two collision points
  - $D^2 > R^2$: No collision (miss)
  - $D^2 = R^2$: Exactly one collision (tangent)

When we have two collision points, we can find the time of collision $t_c$ with this approach:
$$R^2 = | \mathbf{o} + t_c  \mathbf{d} - \mathbf{c} |^2$$
$$R^2 = | t_c  \mathbf{d} - \mathbf{v}|^2$$
This gives a quadratic polynomial in $t$.
$$\mathbf{d}\cdot \mathbf{d} t_c^2 - 2\mathbf{d}\cdot \mathbf{v} t_c + \mathbf{v}\cdot \mathbf{v} - R^2 = 0$$

## Collision between plane and line (old version)
The ray meets the mirror at $$ ar_x(t) + br_y(t) + cr_z(t) = e $$.
See definitions of a,b,c,e below.
Since x,y,z are linear in t, we can solve for t.

    Vector p1, p2, p3;
    p1 = planeOrigin;
    p2 = p1 + planeSideA;
    p3 = p1 + planeSideB;
    double a, b, c, e;
    a = p1.y*p2.z - p2.y*p1.z + p2.y*p3.z - p3.y*p2.z + p3.y*p1.z - p1.y*p3.z;
    b = p1.z*p2.x - p2.z*p1.x + p2.z*p3.x - p3.z*p2.x + p3.z*p1.x - p1.z*p3.x;
    c = p1.x*p2.y - p2.x*p1.y + p2.x*p3.y - p3.x*p2.y + p3.x*p1.y - p1.x*p3.y;
    e = p1.x*p2.y*p3.z - p1.x*p3.y*p2.z + p2.x*p3.y*p1.z - p2.x*p1.y*p3.z + p3.x*p1.y*p2.z - p3.x*p2.y*p1.z;

Ray is parallel if $ad_x + bd_y + cd_z = 0$.
Solve for t to find hitting time t_c:
$$a(o_x+xt_c) + b(o_y+yt_c) + c(o_z+zt_c) = e$$
$$ f = d - ao_x - bo_y - co_z$$
$$t_c = \frac{e}{ad_x + bd_y + cd_z}$$

## Collision between plane and line (new version)
Solve system of equations for t:
$$\mathbf{o}_\mathrm{ray} + t\mathbf{d}_\mathrm{ray} = \mathbf{o}_\mathrm{plane} + \alpha\mathbf{a}_\mathrm{plane} + \beta\mathbf{b}_\mathrm{plane}$$
A solution for $t,\alpha,\beta$ can be found, e.g., with Cramer's rule.