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

```
> # Timid: smallest error among our solutions
> head(dOK[order(dOK$total_absolute_cte),])
   max_runtime   dt reference_speed cte_weight epsi_weight  v_weight
1           90 0.05              50          1 32.02384590 0.6092036
5           90 0.05              50          1  0.16306562 0.8004959
3           90 0.05              50          1  0.09002779 0.8727496
11          90 0.05              50          1  0.07199585 0.7597734
59          90 0.05              50          1 21.85268738 0.6737644
10          90 0.05              50          1  0.20808638 0.8189340
   delta_weight throttle_weight delta_gap_weight throttle_gap_weight crashed
1    4.46984276       0.2217726         833.4191           0.1247289   FALSE
5    0.06660825       0.2277583        1159.8454           0.2108857   FALSE
3    0.86883336       0.1778646        1065.2655           0.1422173   FALSE
11   0.48275760       0.2048286        1283.0884           0.1609739   FALSE
59  19.16793491       0.2018953        1094.3281           0.4070019   FALSE
10  18.99809940       0.2100364        1247.8385           0.2185488   FALSE
   runtime distance total_absolute_cte
1  90.0369  1910.06            14.1555
5  90.0877  1911.28            15.3508
3  90.0200  1913.71            15.3942
11 90.1323  1913.67            15.7798
59 90.0218  1908.25            16.2166
10 90.0277  1911.11            16.2525
> # Aggressive: fastest laps
> head(dOK[order(-dOK$distance),])
   max_runtime   dt reference_speed cte_weight epsi_weight  v_weight
19          90 0.05              50          1  0.48942483 0.8635292
64          90 0.05              50          1  1.36395097 0.5828646
25          90 0.05              50          1  0.26740834 0.8637655
22          90 0.05              50          1  2.90027620 1.0071999
9           90 0.05              50          1  0.09506206 0.8464539
6           90 0.05              50          1  1.14880705 0.8859923
   delta_weight throttle_weight delta_gap_weight throttle_gap_weight crashed
19    0.7391599      0.15809261        1205.4096           0.1897028   FALSE
64    3.9846583      0.09862498        1098.3177           0.1437580   FALSE
25  143.1512323      0.19321711        1248.9559           0.1886518   FALSE
22    7.2168487      0.18224109         647.1699           0.1732579   FALSE
9     0.6336257      0.17103902         695.9972           0.1509335   FALSE
6     1.7941819      0.21369379         853.2799           0.1392649   FALSE
   runtime distance total_absolute_cte
19 90.1670  1916.60            18.1583
64 90.0928  1915.05            24.4139
25 90.1307  1914.88            20.8281
22 90.1831  1914.70            57.4043
9  90.1438  1914.21            36.7277
6  90.1031  1913.94            20.2169
> head(dOK[order(-dOK$distance),])
```
