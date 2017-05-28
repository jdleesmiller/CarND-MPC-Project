Profiling (with -O3 -g) showed
- 87.5% of time spent in MPC Update
- 87.1% of time spent in ipopt::solve
- 10% of time spent in solve_callback

So, most of our time is spent in the solver. Latency for that run was ~0.04.

// initially, we've transformed the waypoints into vehicle coordinates.
// The car is oriented by definition along the +ve x axis.
// So the CTE is just the `y` component.
// As the car turns, that is no longer true.
// The psi here is the steering angle in the car's coordinate system, so
// if we find the angle perpendicular to psi, and then find the distance to
// the polynomial along that line, that's our CTE.
// We also need to know our error in psi, which we could obtain from the
// slope of the polynomial at the intersection point.

With no -O and the new scheme:
ok=1 cost= 143.961 latency=0.0535216
ok=1 cost= 143.289 latency=0.0542186
ok=1 cost= 142.802 latency=0.0543023
ok=1 cost= 142.041 latency=0.0569023
ok=1 cost= 141.874 latency=0.0566871
ok=1 cost= 141.928 latency=0.0554589
ok=1 cost= 141.556 latency=0.0562724
ok=1 cost= 141.181 latency=0.0565127
ok=1 cost= 140.018 latency=0.0605907

With no -O and the old scheme:
ok=1 cost= 132.522 latency=0.0639664
ok=1 cost= 132.454 latency=0.0635567
ok=1 cost= 132.304 latency=0.0637929
ok=1 cost= 132.175 latency=0.0639411
ok=1 cost= 132.283 latency=0.0622105
ok=1 cost= 132.071 latency=0.0632372
ok=1 cost= 132.157 latency=0.0617456
ok=1 cost= 132.326 latency=0.059855
ok=1 cost= 132.114 latency=0.0608867
ok=1 cost= 135.294 latency=0.0590292
ok=1 cost= 136.238 latency=0.0615722
ok=1 cost=  141.01 latency=0.0602917
ok=1 cost= 142.612 latency=0.0610707
ok=1 cost= 142.419 latency=0.0598278

So, a savings of ~10ms. Latency with -O3 was similar --- around ~0.04. So, not
a massive change.
