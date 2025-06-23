# Technical Reflection

<!--toc:start-->
- [Technical Reflection](#technical-reflection)
  - [Bidirectional Communication](#bidirectional-communication)
  - [State Synchronization](#state-synchronization)
  - [Environmental \& Object Interaction](#environmental--object-interaction)
  - [Challenges with TurtleBot](#challenges-with-turtlebot)
  - [Challenges with Development Environment](#challenges-with-development-environment)
  - [Challenges with RViz navigator](#challenges-with-rviz-navigator)
  - [Login details for lab laptops](#login-details-for-lab-laptops)
<!--toc:end-->

> Critical analysis of what was implemented, what was not,
> challenges faced, and future work.

## Bidirectional Communication

Physical-to-digital communication was implemented by the TurtleBot sending its
LiDAR data to its digital twin. This data was used to direct the point-to-point
navigation of the TurtleBot. It was not used to direct which line the TurtleBot
was sent to, as we resrted to only working with a single bus line. This is one 
of the most important features we would want to add, if given more time.

Digital-to-physical communication was implemented by using demand data to guide
the initial stop a TurtleBot is assigned, and then publishing a goal pose to that 
stop. We could have considered more variables in assigning these, but as
the current level is sufficient for proving the concept, we would only implement
this if we had more than the theoretical 1 to 5 weeks.

The reason why we did not implement certain features mentioned in the parapgraphs 
above is that we were too busy trying to solve a more critical issue addressed in 
the next section. We faced no additional challenges trying to implement them, as 
we did not try in the first place since our attention was elsewhere.

## State Synchronization

We successfully tracked the state of the digital robot, but we were unable to do
the same using the physical TurtleBot. We are still unsure about the root cause,
but a misconfigured system clock and digital TurtleBot were part of it. This
would be our number one priority for future improvement, as without this, it is
difficult to demonstrate the system's other features to stakeholders, as we
would only have digital demonstrations.

## Environmental & Object Interaction

We modelled environmental interaction in the digital environment by simulating
passengers leaving the bus stop (because they boarded the bus) whenever the 
TurtleBot got near. We could expand this by allowing for passengers to indicate 
at which stop they want to get off, and whether they want to transfer to another 
line from there. This is not part of our requirements (yet), but it would make 
the system much more useful to stakeholders, so we could consider it for future
improvement.

## Challenges with TurtleBot

As mentioned above, we encountered an issue when synching the Physical TurtleBot
to the Digital Twin. We only received one "frame" of data in RViz and Unity. We 
did not have time to fix it, as replicating the issue was inconsistent and we 
thus lacked understanding of what the cause for it was. During the last
(filming) lab session we identified that the problem was likely due to a clock
desync. Unfortunately, nobody knew how to resolve the issue, so we ended up not
utilizing the Physical TurtleBot.

## Challenges with Development Environment

At first, we wanted to use Docker to avoid the slow speed of VM and the large 
amount of storage space it takes. Unfortunately, installing Docker came with
errors that we were unable to fix for so long that that it was irresponsible to
keep trying to use Docker over VM. Asking tutors for help did not help. So,
in week 3, we made a decision to switch to VM instead. This lead to significant
loss of pace that was felt through the course as we had only 4 more lab sessions 
to implement our PoC.

The VM itself proved incredibly unstable, and we were unable to find any 
fixes that improved performance sufficiently. Different team members were getting
different results for seemingly the same tests, which made resolving issues
significantly harder, as they were hard to replicate. Although we all managed 
to install and run the virtual machine and Unity project in it, work with VM (and 
so also progress) was slow since we averaged 1.5fps when Unity and RViz were
running simultaneously.

## Challenges with RViz navigator

The navigator itself sometimes created winding paths straight through walls present
on the costmap. We suspect that the robot constantly overcompensates, which leads to
it moving in circles. A likely cause is that the friction of one of the wheels is
lower that that of the other, which leads to a different turning radius between the
wheels. However, we were unable to identify the correct friction values.In addition, 
we did not have time to dig through the documentation to find ways to make it "more 
confident" in its path finding. All of aforementioned problems seemed to dramatically 
increase when we sized up the Unity scene appropriately. We tried changing the robot's
footprint in the folder, as explained in [our documentation](/docs/Connecting%20Unity%20to%20Turtlebot.md#changing-the-size-of-the-robot-in-rviz),
but that did not seem to help.

## Login details for lab laptops

Username: ubuntuhost
Password: otherwise-liverwurst-revolver-gamekeeper
