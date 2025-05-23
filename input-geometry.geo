# $ray o=0,0,0 d=0,0,1 e=1 lambda=550e-9
# $parallelRays direction=1,0,0 first=0,-0.2,-1 last=0,-0.2,1 steps=21 e=1
# $parallelRays direction=1,0,0 first=0,-0.1,-1 last=0,-0.1,1 steps=21 e=1
# $parallelRays d=1,0,0 first=0,0,-0.5 last=0,0,0.5 steps=21 e=1 lambda=420e-9
$parallelRays d=1,0,0 first=0,0,-0.5 last=0,0,0.5 steps=21 e=1 lambda=550e-9
# $parallelRays d=1,0,0 first=0,0,-0.5 last=0,0,0.5 steps=21 e=1 lambda=700e-9
# $parallelRays d=1,0,0 first=0,0,-1 last=0,0,1 steps=21 e=1
# $parallelRays direction=1,0,0.1 first=0,0,-0.5 last=0,0,0.5 steps=21 e=1
# $parallelRays direction=1,0,-0.1 first=0,0,-0.5 last=0,0,0.5 steps=21 e=1
# $parallelRays direction=1,0,0 first=0,0.1,-1 last=0,0.1,1 steps=21 e=1
# $parallelRays direction=1,0,0 first=0,0.2,-1 last=0,0.2,1 steps=21 e=1
# $sphericalLens o=8,0,0 r=1 n=1.33
# $sphericalLens o=1,0,0 r=1 n=1.33
# $sphericalLens o=4,0,0 r=1 material=Water
# $sphericalLens o=10,0,0 r=1 n=1.33
# $sphericalLens o=12,0,0 r=1 n=1.33
# $sphericalLens o=14,0,0 r=1 n=1.33
$convexLens o=5,0,0 r=1 n=1.5 h=-0.2,0,0
# $mirror o=5,-1,0 a=0,5,0 b=0,0,5 reflectance=0.5
# $parabolicMirror o=5,0,0 h=-1,0,0.25 c=0.1 reflectance=0.9
