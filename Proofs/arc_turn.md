---
title: "Herbie Skeleton Arc Turn Calculations"
author: Ryan Ozawa
date: June 5, 2021
output:
  pdf_document:
    path: /Project-Herbie/arc_turn.pdf
---

### Given Information

$$
x_1 = 310mm = 0.310m;
x_2=400mm = 0.40m;
y = 556mm = 0.556m
$$

![Herbie Dimension Layout](herbie-measure-layout.png)

We will denote $r$ as the distance from center rover to center of the circle that the rover travels along during its turn.

Each wheel has to have a custom speed and angle to the center of the circle that the rover travels along to actually follow the path properly. To employ this fact we will be using the arc length formula.

$$S = R \theta$$(1)

First derivative of (1) is the velocity equation, since $speed = \frac {distance}{time}$ we just make sure that all wheels travel the same distance in the same time of driving. Thus time is equal for each wheel so we will leave off the time denotion for velocity and also know that each wheel needs to turn the same amount of degrees $\theta$ in this time so velocity is a function of radius R.

We choose to set **Wheel 2** to the max speed and thus each wheel will drive in relation to **Wheel 2** and velocity setting for each wheel can be found by:

$$V_i = V_2 * \frac {R_i}{R_2}$$ (2)

If we say that $V_2 = 1$, we can find the fraction of the speed that each wheel will drive at. With the following radius calculations:

$$
R_1 = R_3 = \sqrt{(\frac {y}{2})^2 + (r + \frac{x_1}{2})^2};
R_2 = R + \frac {x_2}{2}; R_4 = R_6 = \sqrt{(\frac {y}{2})^2 + (r - \frac{x_1}{2})^2}
$$

Plugging this into (2) we get the following results:

$$
V_1 = \frac {\sqrt{(\frac {y}{2})^2 + (r + \frac {x_1}{2})^2}}{r + \frac {x_2}{2}};V_2 = 1; V_3 = \frac {\sqrt{(\frac {y}{2})^2 + (r + \frac {x_1}{2})^2}}{r + \frac {x_2}{2}};
$$

$$
V_4 = \frac {\sqrt{(\frac {y}{2})^2 + (r - \frac {x_1}{2})^2}}{r + \frac {x_2}{2}};
V_5 = \frac {r - \frac {x_2}{2}}{r + \frac {x_2}{2}};
V_6 = \frac {\sqrt{(\frac {y}{2})^2 + (r - \frac {x_1}{2})^2}}{r + \frac {x_2}{2}}
$$

Which simplifies down to:

$$
V_1 = V_3 = \frac {\sqrt{r^2 + 0.31r + 0.1013}}{r + 0.2};V_2 = 1
$$

$$
V_4 = V_6 = \frac {\sqrt{r^2 - 0.31r + 0.1013}}{r + 0.2};
V_5 = \frac {r - 0.2}{r + 0.2}
$$

Since the speeds of each wheel are dependent on the radius of the circle, r. We need to find the physical/software bounds of operation.

$$
tan(\theta) = \frac {\frac {y}{2}}{X}
$$

$$ r\_{min} = X + \frac {x_1}{2} $$(3)

The physical limits of the wheels is a 45 degree therefore:

$$
r_{min} = \frac {x_1}{2} + \frac {\frac {y}{2}}{tan(\theta)} = \frac {0.31m}{2} + \frac {\frac {0.556}{2}}{tan(45)} = 0.433m
$$

Since we don't want to drive the wheels to physical limits all the time, we will back off of it and set the lower bound of R to be 0.45m.

The maximum R that the rover can support will be where the resolution of the encoders can't keep up with the change in degrees. The degree resolution of the encoders was experimentally found to be 0.04 $\frac {degree}{encoder}$. Using (3) but substitute the encoder resolution for $\theta$:

$$
r_{max} = \frac {0.31}{2} + \frac {\frac {0.556}{2}}{tan(0.04)} = 398.36
$$

For the same reason we will back down on this bound to create an upper bound of 300m.

So R is bounded by $0.45m \leq R \leq 300m$.
