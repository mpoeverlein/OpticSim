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
The ray meets the mirror at $$ ar_ x(t) + br_ y(t) + cr_ z(t) = e $$.
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

Ray is parallel if $ad_ x + bd_ y + cd_ z = 0$.
Solve for t to find hitting time $t_c$:
$$a(o_ x+xt_ c) + b(o_ y+yt_ c) + c(o_ z+zt_ c) = e$$
Then,
$$ f = d - ao_ x - bo_ y - co_ z$$
Finally,
$$t_ c = \frac{e}{ad_ x + bd_ y + cd_ z}$$

## Collision between plane and line (new version)
Solve system of equations for t:
$$\mathbf{o}_ \mathrm{ray} + t\mathbf{d}_ \mathrm{ray} = \mathbf{o}_ \mathrm{plane} + \alpha \mathbf{a}_ \mathrm{plane} + \beta\mathbf{b}_ \mathrm{plane}$$
A solution for $t,\alpha,\beta$ can be found, e.g., with Cramer's rule.

## Collision between line and cylinder
The line is given by $$\mathbf{r}_ l(t) = \mathbf{o}_ l + t  \mathbf{d}_ l$$. 
The cylinder is given by $$\mathbf{r}_ c(\alpha) = \mathbf{o}_ c + \alpha  \mathbf{d}_ c$$ with a radius $r$. 

The distance between the line and the cylinder as a function of time is given by:
$$ D(t) = \frac{|\mathbf{r}_ l(t)\times \mathbf{d}_ c|}{|\mathbf{d}_ c|}$$

The numerator can be expressed as:
$$\mathbf{r}_ l(t)\times \mathbf{d}_ c = (\mathbf{o}_ l\times \mathbf{o}_ c)\times \mathbf{d}_ c + t (\mathbf{d}_ l \times \mathbf{d}_ c) = \mathbf{A} + t\mathbf{B}$$
This leads to:
$$t^2  |\mathbf{B}|^2 + t\cdot 2(\mathbf{A}\cdot\mathbf{B}) + |\mathbf{A}|^2 - D^2_{\mathrm{target}} |\mathbf{d}_ c|^2$$