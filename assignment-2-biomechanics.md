**Biomechanics Background and Initial Specifications:**

Team Members: Viraj Kanchan, Nathan Mayer, Shubang Mukund, Matthew Nolan

**Bio Inspiration:**

Our candidate mechanism was chosen to be a dog due to multiple reasons.
Primary reason being the extremely highly and easily available research
papers on the subject. Also, with the advancement in robotics multiple
researchers have produced functional dog like robots thus making it
abundantly clear that the problem is achievable, also it doesnΓÇÖt hurt
that in case of us getting stuck we might seek some help/inspiration
from the pioneers that have done the work before us. That being said,
the team has yet to encounter a canine-like robot that is origami based
or utilizes the axioms of foldable technology. Thus, keeping the problem
challenging. It also helps that various dogs of different breeds have
extremely varying sizes, thus giving us enough wiggle room to check for
distinctive design configurations.

List of Research References:

-   Objective Gait analysis \[1\]\*

-   Three-Dimensional Kinematics of Canine Hind Limbs \[2\]

-   Development of a Canine Rigid Body Musculoskeletal Computer Model to
    > Evaluate Gait \[3\]\*

-   Force development during sustained locomotion \[4\]

-   Relationships of body weight, body size, subject velocity, and
    > vertical ground reaction forces in Trotting Dogs \[5\]

-   Recent developments in canine locomotor analysis: A review \[6\]\*

-   Effect of dog breed and body conformation on vertical ground
    > reaction forces, impulses, and stance times \[7\]

-   Gait evaluation in hip osteoarthritic and normal dogs using a serial
    > force plate system \[8\]

-   Comparison of temporospatial and kinetic variables of walking in
    > small and large dogs on a pressure-sensing walkway \[9\]\*

The paper, ΓÇ£Recent developments in canine locomotor analysis: A reviewΓÇ¥
is as the name suggests an overview of the various kinetic and kinematic
analytical techniques. It outlines and describes each technique in a
concise paragraph and then goes on to discuss the shortcomings and
challenges faced in the existing methodologies and/or models. Not only
this, the paper goes on to talk about methods that can/would be
available in the future. Apart from the typical pressure and force
sensing methods the paper discusses neural signal sensing and FEA
models. This paper is a good place to start and get an understanding of
what kinds of sensing systems are out there. It also provides a quick
launch pad as it has multiple studies referenced throughout which help
us see the bigger picture faster. \[6\]

The paper, ΓÇ£Objective Gait AnalysisΓÇ¥ lays the groundwork by defining all
the parameters and technical abbreviations that are used in the study of
canine gait cycles. It clarifies the difference between a pressure Force
plate sensor and a pressure walkway sensor (see Fig. 1.1). Furthermore,
it differences between Kinetic and Kinematic analysis and how Kinetic
parameters are used in Kinematic analysis. The most crucial part of this
chapter would be the graphs that compare the contribution of the front
and hind legs throughout a complete gait cycle. (Fig 1.2) \[1\].

<img src="media/image1.png" style="width:2.97396in;height:2.17804in" />

Fig. 1.1

<img src="media/image5.png" style="width:2.99648in;height:2.19041in" />

Fig. 1.2

The paper, ΓÇ£Development of a Canine Rigid Body Musculoskeletal Computer
Model to Evaluate GaitΓÇ¥, follows a team of researchers attempting to
create a musculoskeletal model of canine physiology. The data collected
comes from a series of tests performed on an adult, female dachshund.
The team was then able to derive the forces on the body of the dog and
the resulting moments within the leg joints. Therefore, calculations of
power could be determined as the dog performed various gaits. The teamΓÇÖs
results found that the maximal power observed under a normal walking
gait, particularly at the hip, was approximately 0.8 W/kg. This
information along with the forces and moments found throughout the study
will prove extremely helpful as our team makes selections for batteries,
motors, etc. \[3\]

The paper, ΓÇ£Comparison of temporospatial and kinetic variables of
walking in small and large dogs on a pressure-sensing walkwayΓÇ¥ is a
comparative study between the intrinsic behavioral walking patterns,
which the authors refer to as Temporospatial variables (TSVs), like gait
velocity, stride length etc. of dogs from different breeds. This gives
us a good idea of the difference in the walking patterns of different
dogs. One can clearly see the difference between the gait frequency and
the peak vertical force in the graph below (fig 2.1). The conclusion
that the authors offered was that the normalized peak force remained
within a fairly close range. This was despite the fact that the TSVs,
particularly the gait frequency, of the breeds were different. Thus, the
takeaway from this would be that the Kinematic principles of a large dog
could serve as a foundation and a solid starting ground to develop the
kinematic model of the dog-like-robot.
The same paper also gives us some crucial data on the vertical impulse
(VI) and maximum peak pressure(PCP) as shown in the table below (fig
2.2) \[9\]

<img src="media/image7.png" style="width:6.44271in;height:2.22917in" />

Fig 2.1

<img src="media/image4.png" style="width:6.5in;height:1.75in" />

Fig 2.2

**Other Bio-Inspired Robots:**

List of Research References:

-   Mechanical Construction and Computer Architecture of the Four-Legged
    > Walking Machine BISAM \[10\]

-   Design of a Quadruped Robot Driven by Air Muscles \[11\]\*

-   Design and Analysis of Serial Mechanical Leg of Quadruped Lunar
    > Robot \[12\]\*

-   Analysis and research of quadruped robotΓÇÖs legs: A comprehensive
    > review\[13\]\*

-   Exploiting body dynamics for controlling a running quadruped robot
    > \[14\]

In the paper, "Design of a Quadruped Robot Driven by Air Muscles", a
team from the Biorobotics Laboratory at Case Western Reserve University
designed a canine inspired robot, ΓÇ£PuppyΓÇ¥, using a series of pneumatic
actuators. The team attempted to create a mechanical system based on the
biology of a greyhound where each leg in the quadrupedal system is a
three degree of freedom structure consisting of a series of revolute
joints along a two-dimensional plane. The team indicates the dimensions
of the structure while also providing information regarding the maximum
and minimum angles for each joint (Table 1, Fig. 1). Although the
robotic system that will be built this semester will utilize foldable
mechanisms rather than a pneumatic system, the information provided by
the ΓÇ£PuppyΓÇ¥ design team creates an invaluable source of information for
the initial design of the robot. In particular, the foldable mechanisms
created can closely mimic the actuation of the joints described in this
system including the range of motion achieved by each revolute joint
\[11\].

The paper, ΓÇ£Design and Analysis of Serial Mechanical Leg of Quadruped
Lunar RobotΓÇ¥, analyzes a bioinspired robot similar to the previous paper
to be used in unmanned lunar exploration. The team from the Chinese
Academy of Sciences describes the design of their system in great
detail, but also outlines the calculation of forces involved as the
system moves and the reacting torques required at each joint. The
matrices for the system kinematics and resulting Jaconian matrix as
described in the paper will prove invaluable when creating the foldable
mechanisms for the robot. Therefore, the team will be able to create a
similar system and use the predescribed kinematic parameters to
calculate relevant quantities such as forces at the end effector and
torques at each joint \[12\].

In the article ΓÇ£Analysis and research of quadruped robotΓÇÖs legs: A
comprehensive review,ΓÇ¥ a team from School of Mechanical Engineering,
Northwestern Polytechnical University, XiΓÇÖan, China collected research
results of mechanical legs used by quadruped robots and classified them
into three categories (prismatic legs, articulated legs, and redundant
articulated legs) according to the degrees of freedom, introduced and
analyze them. Based on that, they summarized and studied the design
methods of the actuators and mechanical leg structures. They then made
some suggestions for the development of quadruped robotΓÇÖs legs in the
future. \[13\]

Table 1: Parameter Specifications for Initial Design

| **Parameter**              | **Unit**  | **Value Range** | **Reference** |
|----------------------------|-----------|-----------------|---------------|
| Mass of a Greyhound        | Kilograms | 27-40           | \[7\]         |
| Minimum Angle: ≡¥¢╝           | Degrees   | 92              | \[7\]         |
| Minimum Angle: ≡¥¢╜           | Degrees   | 102             | \[7\]         |
| Minimum Angle: ≡¥¢╛           | Degrees   | 106             | \[7\]         |
| Maximum Angle: ≡¥¢╝           | Degrees   | 180             | \[7\]         |
| Maximum Angle: ≡¥¢╜           | Degrees   | 170             | \[7\]         |
| Maximum Angle: ≡¥¢╛           | Degrees   | 192             | \[7\]         |
| Power-to-Weight Ratio      | W/kg      | 0.8 - 2.06      | \[3\], \[4\]  |
| Top speed                  | m/s       | 8-9             | \[4\]         |
| Peak Ground Reaction Force | N/kg      | 1.5-3           | \[4\]         |
| Moment at Hip              | N\*m/kg   | 0.2             | \[3\]         |

**Other Assumptions:**

Although many of the relevant geometric and energy-based parameters are
available in the researched literature, the papers in question all
discuss dogs of varying weights and sizes. In general, however, the
research papers tended to focus on medium and large dogs. Therefore, the
largest assumption that the team will need to make is that the
mass-normalized properties (i.e. peak ground reaction force and
power-to-weight ratio) follow textbook scaling laws so that the figures
may bear some relevance to our smaller system. For example, Taylor \[4\]
compares the peak mass-normalized ground reaction forces of a 30kg dog
and a 130g squirrel, finding that the dog experiences 68% less
mass-normalized force than the squirrel for comparable ground speeds
(1.5 n/kg compared to 4.5 N/kg). An example 1-kg robot system could then
be expected to experience around 4.4 N/kg.

Another key assumption lies in the moments expected at each of the
joints. For instance, \[3\] describes peak moments of 0.2 N\*m/kg at the
hip. Similar scaling laws will need to be assumed to estimate the torque
required at the hip for the small robotic system.

Table 2 : The weight, payload-to-weight ratio, and maximum speed of
significant quadruped robots in recent years

<img src="media/image8.png" style="width:6.5in;height:3in" />

**Figures:**

<img src="media/image3.gif" style="width:1.90038in;height:2.52934in" />

Fig. 1: Depiction of joint placements for the leg of the ΓÇ£PuppyΓÇ¥ robotic
system designed by the Case Western Reserve University Biorobotics
Laboratory \[7\]

<img src="media/image2.png" style="width:2.58732in;height:3.30741in" />

Fig. 2: Force and Moment of the Mechanical Leg \[8\]

<img src="media/image6.png" style="width:2.96354in;height:3.15742in" />

Fig. 3: Mammal-type robot - The joint torque is small when the leg is
bent, and there is almost no joint torque when standing upright\\

<img src="media/image9.jpg" style="width:6.5in;height:3.98611in" />

Fig. 4: Angle, Velocity, Moment, and Power Data Collected for an Adult,
Female Dachshund for a Series of Different Gaits \[9\]

**Engineering Representation:**

The simplest engineering representation of this system consists of a
base body and four limbs, with each limb containing three joints and
three links to facilitate the necessary reciprocating motion. A basic
diagram of the system is shown below with its links, joints, and springs
labeled.

<img src="media/image10.jpg" style="width:3.45313in;height:4.44792in" />

Fig. 5: Simple engineering diagram of the quadruped system

Note that only one of the four legs are shown for simplicity, but all
four legs are identical in structure. There are 3\*4+1=13 links (Link 0
for the body, links 1, 2, and 3 for the first leg, and so on) and 12
joints (3 per leg). Each leg contains two torsional springs as well, at
the second and third joint (S1 and S2). Link 1 (and the corresponding
links of the other legs) may be assumed to be massless as they
contribute very little to the overall mass of the system and do not
represent a large amount of inertia during a normal walking gait since
they are so high up. The main actuator is present at joint 2 in the
above diagram. Although it is not the first joint in the chain, it is
still responsible for swinging the largest amount of leg mass through
the largest range of motion during a walking gait.

**Discussion:**

1.  Discuss / defend your rationale for the size of the animal you
    > selected in terms of your ability to replicate key features
    > remotely with limited material selection.

Being particularly interested in optimizing quadrupedal gaits, the most
obvious choice was to take bio-inspiration from the dog. Part of this
decision related to the abundance of research relating to current
robotic dogs and the possible novelty of using foldable techniques to
construct a dynamical dog system. Given the wide range of dog breeds,
the team has relatively high design freedom with regards to the scale of
the system to be studied. A small-breed dog would require manageably
sized limb mechanisms, and the key features such as limb proportions and
joint stiffnesses should be achievable with limited, laminate materials.
The joint and link stiffnesses should be reproducible through using
stiff laminate links (e.g. thick carbon fiber or fiberglass stack-ups)
and joint materials with a tuned flexural stiffness.

1.  Find a motor and battery that can supply the mechanical power needs
    > obtained above. Consider that motor efficiencies may be as high as
    > 95%, but if you canΓÇÖt find it listed, assume you find a more
    > affordable motor at 50-70% efficiency. Compare the mechanical
    > watts/kg for the necessary motor and battery vs the animalΓÇÖs
    > mechanical power/mass above? Which one is more energy dense?

Based on the data collected in the paper, ΓÇ£Development of a Canine Rigid
Body Musculoskeletal Computer Model to Evaluate GaitΓÇ¥, researchers found
that the maximum power during a normal gait cycle for an adult, female
dachshund was about 0.8 W/kg. Although these values likely vary between
breeds, the team has determined to utilize this value as a basis for
initial calculations \[3\]. A sample servo motor from Sparkfun, weighing
45.5 grams and supplying about 0.8 oz-in of torque would allow the
joints to overcome the necessary moments throughout each gait cycle
based on the provided data. Additionally, the power supply required
would need to provide at least 4.8 Volts to operate the motor; however,
with the theoretical motor efficiency of 70%, the motor would require a
6.8 Volt power supply. When considering the mass to power ratio of both
the biological system and the theoretical motor and battery system, the
canine would be less energy dense as compared to the theoretical model.
The calculations made based on the paperΓÇÖs provided data indicate that
the mechanical power/mass ratio of the proposed system would be about 36
W/kg due to the power required by the motor. When compared to the
maximum power required by the dog of 0.8 W/kg during a normal gait, the
muscleΓÇÖs capabilities to deliver the same forces through a more
distributed power allow the canine physiology to be much less energy
dense \[3\].

Motor:
[<u>https://www.sparkfun.com/products/11884</u>](https://www.sparkfun.com/products/11884)

Battery:
[<u>https://www.sparkfun.com/products/11855</u>](https://www.sparkfun.com/products/11855)

Comparison of temporospatial and kinetic variables of walking in small
and large dogs on a pressure-sensing walkway

Bibliography:

> \[1\] Bryan T. Torres, ΓÇ£Canine Lameness \|\| Objective Gait AnalysisΓÇ¥
> \[Online\]. Available:
> https://onlinelibrary.wiley.com/doi/pdf/10.1002/9781119473992.ch2

\[2\] Fischer, M.S., Lehmann, S.V. & Andrada, E. Three-dimensional
kinematics of canine hind

limbs: in vivo, biplanar, high-frequency fluoroscopic analysis of four
breeds during

> walking and trotting. Sci Rep 8, 16982 (2018).
>
> https://doi.org/10.1038/s41598-018-34310-0
>
> \[3\] N. P. Brown, G. E. Bertocci, G. J. R. States, G. J. Levine, J.
> M. Levine, and D. R. Howland, ΓÇ£Development of a canine rigid body
> musculoskeletal computer model to evaluate gait,ΓÇ¥ Frontiers,
> 01-Jan-1AD. \[Online\]. Available:
> https://www.frontiersin.org/articles/10.3389/fbioe.2020.00150/full.
> \[Accessed: 15-Feb-2022\].
>
> \[4\] C. R. Taylor, ΓÇ£Force development during sustained locomotion: A
> determinant of gait, speed and metabolic power,ΓÇ¥ *Journal of
> Experimental Biology*, vol. 115, no. 1, pp. 253ΓÇô262, 1985.
>
> \[5\] K. Voss, L. Galeandro, T. Wiestner, M. Haessig, and P. M.
> Montavon, ΓÇ£Relationships of body weight, body size, subject velocity,
> and vertical ground reaction forces in Trotting Dogs,ΓÇ¥ *Veterinary
> Surgery*, vol. 39, no. 7, pp. 863ΓÇô869, 2010.
>
> \[6\] Robert L. Gillette, T. Craig Angle, ΓÇ£Recent developments in
> canine locomotor analysis: A review,ΓÇ¥ The Veterinary Journal, Volume
> 178,Issue 2 \[online\] https://doi.org/10.1016/j.tvjl.2008.01.009.
>
> \[7\]Voss, K. et al (2011). ΓÇ£Effect of dog breed and body conformation
> on vertical ground reaction forces, impulses, and stance timesΓÇ¥.
> *Veterinary and Comparative Orthopedics and Traumatology*, 24(2),
> 106ΓÇô112. doi:10.3415/vcot-10-06-0098
>
> \[8\] Kennedy, S., et al. "Gait evaluation in hip osteoarthritic and
> normal dogs using a serial force plate system." Veterinary and
> Comparative Orthopaedics and Traumatology 16.03 (2003): 170-177.
>
> \[9\] Kim, Jongmin, Kris A. Kazmierczak, and Gert J. Breur. "."
> American journal of veterinary research 72.9 (2011): 1171-1177.

\[10\] K. Berns, W. Ilg, M. Deck, J. Albiez and R. Dillmann, "Mechanical
construction and

computer architecture of the four-legged walking machine BISAM," in
IEEE/ASME

Transactions on Mechatronics, vol. 4, no. 1, pp. 32-38, March 1999, doi:

10.1109/3516.752082.

\[11\] K. S. Aschenbeck, N. I. Kern, R. J. Bachmann and R. D. Quinn,
"Design of a Quadruped

Robot Driven by Air Muscles," The First IEEE/RAS-EMBS International
Conference on

Biomedical Robotics and Biomechatronics, 2006. BioRob 2006., 2006, pp.
875-880, doi:

10.1109/BIOROB.2006.1639201.

\[12\] Y. Wu, S. Wang, and K. Wang, ΓÇ£Design and analysis of serial
mechanical leg of quadruped

lunar robot: Proceedings of the 2019 4th International Conference on
Automation, control

and Robotics Engineering,ΓÇ¥ ACM Digital Library, 01-Jul-2019. \[Online\].
Available:

https://dl.acm.org/doi/abs/10.1145/3351917.3351947

\[13\] Yuhai Zhong , Runxiao Wang, Huashan Feng and Yasheng Chen :
Analysis and research of quadruped robotΓÇÖs legs: A comprehensive
review\[ Online\]. Available:
https://journals.sagepub.com/doi/pdf/10.1177/172988141984414

\[14\] F. Iida, G. Gomez, and R. Pfeifer, ΓÇ£Exploiting body dynamics for
controlling a running quadruped robot,ΓÇ¥ *ICAR '05. Proceedings., 12th
International Conference on Advanced Robotics, 2005.*, Sep. 2005.
