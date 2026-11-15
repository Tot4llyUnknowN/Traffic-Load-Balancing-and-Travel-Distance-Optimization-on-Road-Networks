## A LAB REPORT 

ON 

# **Traffic Load Balancing and Travel Distance Optimization on Road Networks** 

_Submitted by_ Kazi Shafayet Reza Md. Imran Faraj Roll: 2307020 Roll: 2307021 

_In_ 

_Computer Science and Engineering_ Second Year Second Term 

Course No: CSE-2202 

Course Title: Algorithm Analysis and Design 

1 

## **<u>Problem Statement:</u>** 

Think of a city as a road map. Intersections are dots, roads are lines, and the number on each road is its length. At every time step some groups of vehicles show up. Each group tells us: “there are _n_ vehicles, they start at intersection _s_ and want to get to intersection _d_ .” We have to choose their routes, and we are allowed to **split a group over several routes** . 

The two goals pull in opposite directions. Shortest routes give the smallest total distance but jam a few roads and leave the rest empty. Spreading vehicles out relieves the busy roads but makes routes longer, so we need a compromise. We also have to report the routes, measure them, compare against shortestpath and greedy routing, and analyze the cost. 

#### **Considerations taken:** 

- Every generated network is connected (random spanning tree first, extra roads after), so every request has at least one path. 

- Every path is simple and goes from _si_ to _di_ . The vehicles assigned to a request always add up to _ni_ , and vehicles are counted as whole numbers. 

- A route can be only a limited amount longer than the shortest one (the detour cap), otherwise balancing would send cars on silly long trips. 

- All algorithms use the same networks, requests and seeds, with a small portable random generator (SplitMix64) so runs repeat on any compiler. 

- We treat time steps as independent, so each step starts with empty roads. 

- The congestion penalty scales with the average road length and group size, so one setting works for small and large networks. 

- If the balancing search fails, we fall back to the shortest path so every request is still served. 

#### **Mathematical formulation:** 

The network is a connected, undirected graph _G_ =( _V , E ,w_ ) with _w e_ >0. At time step _t_ the requests are _r i_ =( _ni ,si ,di_ ). Let _Pi_ be the set of simple _si_ – _di_ paths,<sup>_l_(</sup><sup>_p_)=</sup> ∑ _we_ , and let _d_ ( _si ,di_ ) be the shortest _e ∈ p_ 

distance. Let _xi, p ∈_ ℤ _≥_ 0 be the number of vehicles of request _i_ sent on path _p_ : 



The four metrics we use to evaluate a solution are 



This is a problem with two objectives: we want to minimize ( _Lmax_ or _Var_ ( _L_ ), _D_ ) subject to the detour cap _l_ ( _p_ ) _≤_ (1+ _δ_ ) _d_ ( _si ,di_ ) whenever _xi, p_ >0. Shortest-path routing reaches the smallest possible total distance<sup>_DSP_=∑</sup> _ni d_ ( _si ,di_ ) but completely ignores load. We use the distance ratio _D_ / _D SP_ to _i_ compare algorithms. One standard way to combine both goals is the convex potential 



2 

The second term of this potential penalizes traffic that is concentrated on a few roads ( 

1 _L_ 2 _e_ = _Var_ ( _L_ )+ ´ _L_ 2). With whole-number, unsplittable flows the congestion problem is NP-hard, so | _E_ |<sup>∑</sup> _e_ we use a heuristic instead of solving it exactly. Keep in mind that _L_<sup>´</sup> counts road segments and not kilometres, so it can go down even while _D_ goes up. The metrics that really matter are _Lmax_ , _Var_ ( _L_ ) and _D_ . 

## **<u>Approach and Algorithm:</u>** 

The algorithm we propose is **CAMR-SF** (Corridor-Aware Marginal-cost Router). Here is how it works in simple terms: 

1. **Busy roads feel longer.** Every road gets a “feels-like length”: its real length plus a penalty that grows with the number of cars already on it. A busy road therefore looks longer, and new cars try another way. 

2. **Detour corridor.** A group may only use intersections that keep its whole trip within 50% of the shortest distance. 

3. **Platoons.** Each request is cut into up to 6 platoons, which are sent in rounds (every request gets a turn before any request gets a second one). A platoon takes the cheapest feels-like route, and that changes the loads the next platoon sees. 

4. **Second look.** Once all the others are placed, each platoon is re-routed. The new route is kept only if it is cheaper than the old one. 

5. **Fix the worst road.** We find the busiest road and move platoons off it, but only onto routes where no road reaches the current peak. 



<!-- Start of picture text -->
© Setup 1 Split 2 Route 3 Refine Peak shav 5 Report<br>shortest |-®| cut requests |-® | cheapest path |-® | re-route; keep |-® move groups off-> | merge paths,<br>distances + into <6 in corridor, if cheaper busiest road output loads.<br>corridors platoons. round-robin (s 6 passes) (= 50 moves) and metrics<br><!-- End of picture text -->

Figure 1. The five stages of CAMR-SF (the blue boxes are the balancing stages). 

3 



<!-- Start of picture text -->
ogJ >)204 LacedLeer way eng ound ength20 14 empty)<br>B57 soup 3:2,<br>3 20 a<br>3, [sew<br>er ee ee<br>Vehicles aeady onthe direc oad<br><!-- End of picture text -->

Figure 2. Why a busy road feels longer. A request can use a direct road (length 10) or a detour (length 14). Platoons have 10 vehicles and the load penalty is 0.5 per vehicle. The first two platoons take the direct road; once it carries 20 vehicles it feels longer than the detour, so the third platoon switches. 

In mathematical terms, let _w_ ´ be the mean road length and _n_ ´ the mean group size. The congestion factor, and the cost of using road _e_ for a platoon of _q_ vehicles, are 



With a detour allowance _δ_ (default 0.5), the corridor and the length budget of request _i_ are 



and a search step to node _v_ is allowed only if (length so far) + _d_ ( _v ,di_ ) _≤Λi_ . Request _i_ is split into _mi_ = _min_ ( _P ,ni_ ) platoons ( _P_ =6) whose sizes differ by at most one. The phases are then: 

- **Phase 1:** in round _r_ =1.. _P_ , we take the requests in decreasing order of _ni_ , route platoon _qir_ on the cheapest corridor path under _ce_ ( _qir ; L_ ), and update _L_ . 

- **Phase 2 (at most 6 passes):** take a platoon’s own load off the network, find its cheapest corridor path again, and switch only if _c_ ( _p_ new )< _c_ ( _p_ old ). 

- **Phase 3 (at most 50 moves):** let _e_ * be the busiest road, with load _L_ *. For each platoon on _e_ * we look for the cheapest corridor path with _Le_ + _q_ < _L_ * on every road, and apply the move with the smallest cost increase. 

## **<u>Pseudocode:</u>** 

#### **Algorithm 1: CAMR-SF.** cost(path, q) = sum over e in path of [ w(e) + k (L(e) + q/2) ] 

Input: G=(V,E,w); requests {(n_i, s_i, d_i)}; alpha, P=6, delta=0.5, p_max=6, m_max=50 

1. wbar = mean w(e); nbar = mean n_i; k = alpha * wbar / nbar; L(e) = 0 for all e. 

2. For each request i: run Dijkstra from s_i and from d_i (stopped beyond (1+delta)*d(s_i,d_i)). Lambda_i = (1+delta)*d(s_i,d_i); C_i = { v : d(s_i,v) + d(v,d_i) <= Lambda_i }. Split n_i into m_i = min(P, n_i) platoons of nearly equal size. 

3. Phase 1: for r = 1..P: for each request i in decreasing order of n_i (if r <= m_i): path = CorridorRoute(i, q_ir, L, cap = infinity); if path is empty, path = shortest path. L(e) += q_ir for e in path; store (i, q_ir, path). 

4. Phase 2: repeat up to p_max times: 

for each stored batch (i, q, path): remove q from L on path; cand = CorridorRoute(i, q, L, infinity); 

if cand exists and cost(cand, q) < cost(path, q): path = cand. add q back to L on path. 

4 

stop when no batch moved. 

5. Phase 3: repeat up to m_max times: 

e* = busiest road; Lstar = L(e*); best = none. 

for each batch (i, q, path) with e* in path: remove q from L on path; 

cand = CorridorRoute(i, q, L, cap = Lstar); 

if cand exists: keep the candidate with the smallest cost(cand,q) - cost(path,q); 

add q back to L on path. 

if best = none: stop; else move that batch to its cand path. 

6. Merge batches of one request that use the same path; output paths, vehicle counts and L. 

#### **Algorithm 2: CorridorRoute(i, q, L, cap)** (Dijkstra with feels-like cost) 

1. cost[s_i] = 0; len[s_i] = 0. Repeatedly take the unfinished node u with smallest cost. 

2. For each road e = (u,v): skip if v is not in C_i; skip if L(e) + q >= cap; 

newlen = len[u] + w(e); skip if newlen + d(v, d_i) > Lambda_i; c = cost[u] + w(e) + k*(L(e) + q/2); if c < cost[v]: update cost, len, parent of v. 

3. Return the path from s_i to d_i via parents, or "empty" if d_i is not reached. 

**Baselines** (same Dijkstra, different edge cost): 

SP    : all n_i vehicles of every request on a shortest path (cost w(e)). GLB-U : requests by decreasing n_i; each request goes whole on ONE path minimizing sum over e of [ (L(e)+n_i)^2 - L(e)^2 + 0.05*w(e) ]; update L. GLB-S : each request is cut into platoons of size ceil(n_i/6); same cost with c in place of n_i; platoons are routed round-robin over requests. No corridor, refinement or peak shaving. 

## **<u>Proof of Correctness:</u>** 

**Lemma 1 (potential).** If we add _q_ vehicles along a path _p_ , the potential _Φ_ changes by 



the feels-like cost is exactly the increase in potential per vehicle. This means Phase 1 is a greedy step on _Φ_ . 

**Lemma 2 (Phase 2 terminates).** Take the loads with the platoon’s own vehicles removed ( _L_ -). Moving the platoon from _p_ to _p'_ changes _Φ_ by exactly _q_ [ _c_ ( _p' ;q , L_ -)− _c_ ( _p;q , L_ -)]. Phase 2 only accepts moves where this value is negative, so _Φ_ strictly decreases every time. _Φ_ cannot go below its minimum and there are only finitely many assignments, so the phase must stop, in a state where no single platoon can lower its own cost. 

**Lemma 3 (peak shaving never creates a new peak).** On the new path every road satisfies _Le_ + _q_ < _L_ *, the old path only loses load, and _e_ * drops to _L_ *− _q_ . So the maximum load never goes up, and the number of roads at the peak value goes down with every move. 

**Lemma 4 (detour cap and conservation).** The search enforces the budget _Λi_ , and the fallback shortest path satisfies it too, so every path is at most (1+ _δ_ ) _d_ ( _si ,di_ ) long. The platoon sizes add up to _ni_ and each platoon gets exactly one path, so no vehicle is lost. 

_Check by code:_ we recomputed 3,669 paths from 15 time steps on 500-intersection instances and found 0 violations of path validity, the detour cap, vehicle conservation or load consistency. 

5 

**Conclusion of the proof:** CAMR-SF always returns a valid assignment within the detour cap. It is still a heuristic: the result depends on platoon order, and we have no optimality guarantee or lower bound. 

## **<u>Results:</u>** 

**Experimental setup (Task 1).** Each network is a random spanning tree plus random extra roads, with road lengths in [2 _,_ 20 ], and each request has _ni ∈_ [20 _,_ 100 ] vehicles and two different random endpoints. The tiers (intersections / roads / requests per step / seeds) are Small 20/45/10/10, Medium 100/280/30/10, Large 500/1500/80/10 and X-Large 1500/4500/150/5, with 5 time steps each (3 for X- Large). A small demo network has 8 intersections and 14 roads over 3 time steps. The settings are _α_ =1, _P_ =6, and _δ_ =0.5. Times are measured single-threaded on one machine, so they are only good for comparing the algorithms. **SP** is plain shortest-path routing, **GLB-U** is an unsplittable greedy balancer, and **GLB-S** is a splittable greedy balancer (6 platoons, no distance limit), the strongest pure balancing baseline. 

**Paths, splits and loads on the demo network (Tasks 3 and 4).** 

**Table 1.** Paths chosen by CAMR-SF (vehicles in brackets). 

|**t**<br>**Request**|**Paths (vehicles)**|
|---|---|
|1<br>r1: 60 veh, 0→7|0→3→5→7 (40); 0→2→5→6→7 (10); 0→2→5→7 (10)|
|r2: 40 veh, 1→6|1→4→6 (34); 1→4→5→6 (6)|
|r3: 50 veh, 0→6|0→2→4→6 (17); 0→1→2→5→6 (16); 0→3→5→6 (9); 0→2→5→6 (8)|
|2<br>r1: 70 veh, 0→5|0→2→5 (35); 0→3→5 (24); 0→1→4→5 (11)|
|r2: 45 veh, 2→7|2→5→7 (16); 2→4→6→7 (15); 2→4→5→7 (14)|
|r3: 55 veh, 1→7|1→4→5→7 (19); 1→2→4→6→7 (18); 1→4→6→7 (18)|
|r4: 30 veh, 3→6|3→5→6 (30)|
|3<br>r1: 80 veh, 0→7|0→1→4→6→7 (26); 0→2→4→6→7 (26); 0→2→5→7 (14); 0→3→5→7 (14)|
|r2: 65 veh, 1→5|1→2→5 (33); 1→4→5 (32)|
|r3: 40 veh, 3→7|3→5→7 (34); 3→5→6→7 (6)|



**Table 2.** Road loads (vehicles). SP = shortest path at _t_ =1; C1, C2, C3 = CAMR-SF at _t_ =1 _,_ 2 _,_ 3. 

|**Road**|**Len**|**SP**|**C1**|**C2**|**C3**|**Road**|**Len**|**SP**|**C1**|**C2**|**C3**|
|---|---|---|---|---|---|---|---|---|---|---|---|
|0–1|4|50|16|11|26|2–5|6|60|44|51|47|
|0–2|5|60|45|35|40|3–5|4|0|49|54|54|
|0–3|7|0|49|24|14|4–5|3|0|6|44|32|
|1–2|2|0|16|18|33|4–6|5|90|51|51|52|
|1–4|6|90|40|48|58|5–6|6|0|49|30|6|
|2–3|3|0|0|0|0|5–7|5|60|50|49|62|
|2–4|5|0|17|47|26|6–7|3|0|10|51|58|



6 



<!-- Start of picture text -->
SP (busiest road: 90) CAMR-SF (busiest road: 51)<br>tO mom @_<br>50 90; 51<br>( 60) (OO 0 0) >) we 16 47 6<br>to} Oe° poe0 0 On49 =O,45 0 49 10<br>_ 60, >e 4<br>= \y” OO<br><!-- End of picture text -->

Figure 3. Demo network at t = 1. Left: shortest-path routing puts 90 vehicles on two roads and leaves seven roads empty. Right: CAMR-SF uses all but one road, and the busiest road carries 51 vehicles. 

**Table 3.** Metrics for all algorithms on the demo network (Tasks 4 and 5). 

|**Step**|**Algorithm**|**Max load**|**Avg load**|**Variance**|**Total distance**|**Dist / SP**|
|---|---|---|---|---|---|---|
|1|SP|90|29.3|1249.5|2150|1.000|
||GLB-U|60|35.0|539.3|2310|1.074|
||GLB-S|57|32.9|233.6|2314|1.076|
||CAMR-SF|51|31.6|346.2|2296|1.068|
|2|SP|170|32.5|2417.0|2280|1.000|
||GLB-U|70|35.7|635.2|2425|1.064|
||GLB-S|62|36.3|258.6|2411|1.057|
||CAMR-SF|54|36.6|279.8|2433|1.067|
|3|SP|145|32.1|2370.4|2160|1.000|
||GLB-U|80|35.0|1092.9|2385|1.104|
||GLB-S|68|37.6|256.1|2416|1.119|
||CAMR-SF|62|36.3|376.1|2320|1.074|



On the demo network CAMR-SF has the lowest maximum load at every step (51, 54 and 62, against 90, 170 and 145 for shortest path) with distance within 7.4% of shortest path. GLB-S has a lower variance, but CAMR-SF uses the least distance of the three balancing algorithms at _t_ =1 and _t_ =3. 

7 

#### **Comparison on generated networks (Task 5).** 

**Table 4.** Mean over seeds (± SD for max load). Time is per step. 

|**Network**|**Algorithm**|**Max load**|**Variance**|**Avg load**|**Dist / SP**|**Time(ms)**|
|---|---|---|---|---|---|---|
|Small|SP|213.6 ± 29.9|2948.8|32.4|1.000|0.02|
||GLB-U|103.2 ± 10.0|1164.0|29.7|1.312|0.02|
||GLB-S|88.2 ± 11.6|364.8|32.4|1.521|0.11|
||CAMR-SF|127.5 ± 22.5|1218.9|29.8|1.112|0.15|
|Medium|SP|287.9 ± 36.0|2187.6|22.7|1.000|0.23|
||GLB-U|111.8 ± 16.3|955.3|19.5|1.274|0.19|
||GLB-S|87.1 ± 11.7|168.6|22.9|1.648|1.49|
||CAMR-SF|129.8 ± 9.9|675.1|20.7|1.143|2.65|
|Large|SP|308.4 ± 28.8|1345.3|15.0|1.000|3.81|
||GLB-U|118.2 ± 15.4|689.6|12.2|1.234|3.00|
||GLB-S|93.0 ± 14.3|91.7|15.4|1.718|26.89|
||CAMR-SF|122.2 ± 9.8|356.0|13.7|1.150|41.18|
|X-Large|SP|308.5 ± 30.9|943.3|11.1|1.000|21.41|
||GLB-U|119.3 ± 11.8|535.7|8.9|1.220|16.48|
||GLB-S|88.5 ± 16.5|67.9|11.6|1.748|151.77|
||CAMR-SF|122.7 ± 9.8|240.1|10.3|1.151|212.67|





<!-- Start of picture text -->
mem SP mm GLB-U mmm LBS. mm_CAMR'SF<br>Max road load Load variance (log) Distance / shortest path<br>1.8<br>300<br>10? 1.6<br>200<br>14<br>100 1.2<br>10)2 1.0<br>0<br>s M L XL s M L XL s M L XL<br><!-- End of picture text -->

Figure 4. Maximum load, variance (log scale) and distance ratio on Small (S), Medium (M), Large (L) and X-Large (XL) networks. The whiskers show ± 1 SD. 

- **Against shortest path:** CAMR-SF reduces the maximum load by 40% on the Small networks and by 55–60% on the larger ones. Variance drops by 59–75%, and the price is 11–15% more distance. 

- **Against GLB-U:** On Large and X-Large networks the maximum load is within one standard deviation of GLB-U (122.2 vs 118.2 and 122.7 vs 119.3), the variance is about half, and the distance is 6–7% lower. On Small and Medium networks the maximum load is 24% and 16% higher (few parallel routes fit in the corridor), but the distance is still 15% and 10% lower. 

- **Against GLB-S:** GLB-S has the lowest maximum load and variance on every network size (the maximum load of CAMR-SF is 31–49% higher). But its routes are 52–75% longer than shortest ones, compared with 11–15% for CAMR-SF. 

8 

#### **Utilization, congestion and distance.** 

**Table 5.** Road use on one Medium instance (280 roads). 

|**Algorithm**|**Roads used**|**Traffic on busiest 10% of roads**|**Peak / average**|**Max load**|
|---|---|---|---|---|
|SP|84 / 280|52%|12.9×|282|
|GLB-U|99 / 280|47%|5.0×|100|
|GLB-S|275 / 280|20%|3.4×|80|
|CAMR-SF|188 / 280|34%|5.0×|106|





<!-- Start of picture text -->
se<br>250 =~ Guu<br>2 --- Glas<br>3 200 — camer<br>5 iso<br>2 1004 Yn<br>3 Ne<br>0 LS SSS<br>° 50 100 150 200 ~250<br>Roads ranked busiest to quietest (280 roads)<br><!-- End of picture text -->

Figure 5. Roads ranked from busiest to quietest on one Medium instance. 

Shortest-path routing leaves 70% of the roads empty and puts 52% of the traffic on its busiest 10% of roads. Its bottleneck (282) is 2.7–3.5 times heavier than that of the balancing algorithms. CAMR-SF uses 188 roads and has the same peak-to-average load as GLB-U (5.0×). Its distance cost is small and stable (1.11–1.15, SD 0.006–0.024), while GLB-S grows from 1.52 to 1.75 as the network gets bigger. Over five steps on one Large network, GLB-S had the lowest maximum load at four steps. CAMR-SF beat GLB-U at steps 1 and 4 and lost at steps 3 and 5. At step 2 all three load-aware methods tied at 180, which looks like a structural bottleneck (not investigated further). The distance ratio of CAMR-SF stayed within 1.15– 1.17. 

## **<u>Analysis</u>** 

### **Time complexity** 

Let _D_ = _E_ + _V_ log _V_ be the cost of one Dijkstra run, _R_ the number of requests per step and _Dc ≤D_ the cost of one corridor search. 

- SP: _O_ ( _R D_ ). GLB-U: _O_ ( _R_ log _R_ + _R D_ ). GLB-S: _O_ ( _P R D_ ). 

- CAMR-SF setup: _O_ ( _R D_ ) (one shortest-path search and two bounded searches per request). 

- Phase 1: _O_ ( _P R Dc_ ). Phase 2: _O_ ( _pmax P R Dc_ ) (one search per platoon per pass). Phase 3: _O_ ( _mmax B Dc_ ), where _B≤PR_ is the number of platoons on the hottest road. 

- Total: _O_ ( _R D_ +(1+ _pmax_ + _mmax_ ) _P R Dc_ )= _O_<sup>(</sup> _P R D_<sup>)</sup> for fixed constants, which is roughly linear in the network size (up to a log factor) and linear in the number of requests. 

### **Space complexity** 

_O_ ( _V_ + _E_ + _P R_<sup>´</sup> _l_ )<sup>for the graph, the search arrays and the stored paths (´</sup> _l_<sup>is the mean path length).</sup> 

9 

### **Running time and scalability** 

CAMR-SF is 8–12 times slower than shortest-path routing and 1.4–1.8 times slower than GLB-S (Table 4); one X-Large step takes about 0.2 s. The measured scaling exponents are **0.99 in the number of intersections** (100 to 1,600) and **1.02 in the number of requests** (25 to 400), so growth is close to linear. At _δ_ =0.5 only about 15% of intersections and 7.5% of roads are searched on a Large instance; at _δ_ =2 about 90% of the network is searched. 



<!-- Start of picture text -->
© slope = 0.99 2007 slope = 1.02<br>5 20 3 =<br>& &<br><!-- End of picture text -->

Figure 6. CAMR-SF running time per step on log–log axes. A slope of 1 means linear growth. 

### **Trade-off between balance and distance** 

**Table 6.** Medium networks, 10 seeds, _t_ =1 (SP maximum load: 250.5). 

|**Setting**|**Max load **|**Variance **|**Dist / SP**|
|---|---|---|---|
|α = 0|142.1|1371.7|1.024|
|α = 0.5|125.2|706.6|1.092|
|α = 2 (δ = 0.5)|124.1|560.3|1.165|
|δ = 0.1|196.0|1325.3|1.015|
|δ = 0.5 (default, α = 1)|125.0|610.5|1.130|
|δ = 1|90.6|456.5|1.189|
|δ = 2|70.6|379.8|1.248|
|no corridor|67.7|361.5|1.266|
|GLB-U|99.1|892.5|1.267|
|GLB-S|78.0|147.8|1.622|





Figure 7. Maximum load against distance on Medium networks (lower left is better). 

- _δ_ **is the main knob.** Widening the corridor lowers load and variance but raises distance. At _δ_ =1 CAMR-SF beats GLB-U on load (90.6 vs 99.1) and distance (1.189 vs 1.267). At _δ_ =2 its maximum load (70.6) is below GLB-S (78.0) with 23% less distance. 

- _α_ **saturates.** Past about 0.5 the maximum load is almost flat (125.2 vs 124.1 at 2) while distance keeps rising. 

- _α_ =0 **is not shortest path.** The maximum load is still 43% below shortest path, because Phase 3 works on loads whatever _α_ is. 

- **Phases:** splitting into platoons gives most of the variance gain, and peak shaving cuts the maximum a further 8% at almost no extra distance (Table 7). 

10 

**Table 7.** Contribution of each phase (Medium networks, 10 seeds, 5 steps). 

|**Variant**|**Max load **|**Variance **|**Dist / SP**|
|---|---|---|---|
|One batch per request, no|151.1|1075.4|1.128|
|corridor||||
|Phase 1 only (6 platoons, corridor)|144.8|703.8|1.130|
|Phases 1 + 2|140.9|675.1|1.140|
|Phases 1 + 2 + 3 (full)|129.8|675.1|1.143|



## **<u>Conclusion</u>** 

- Compared with shortest-path routing, CAMR-SF cuts the maximum road load by about 40–60% and the variance by 59–75%, for 11–15% more distance, within the detour cap. 

- It does not beat greedy balancing everywhere: it matches GLB-U on larger networks, is weaker on small ones, and balances less than GLB-S but with far shorter routes. 

- Running time grows almost linearly. Limits: CAMR-SF is a heuristic with no lower bound, and the baselines have no distance limit. 

11 

