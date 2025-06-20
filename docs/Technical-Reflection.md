# Technical Reflection

<!--toc:start-->
- [Technical Reflection](#technical-reflection)
  - [Bidirectional Communication](#bidirectional-communication)
  - [State Synchronization](#state-synchronization)
  - [Environmental & Object Interaction](#environmental-object-interaction)
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
was sent to. This is one of the most important features we would want to add, if
given more time.

Digital-to-physical communication was implemented by using demand data to guide
the line and stop a TurtleBot is assigned, and then publishing a goal pose along
that line. We could have considered more variables in assigning these, but as
the current level is sufficient for proving the concept, we would only implement
this if we had more than the theoretical 1 to 5 weeks.

The reason why did not implement certain features in the parapgraphs above was
that we were too busy trying to solve a more critical issue addressed in the
next section. We faced no other challenged trying to implement them, as we did
not try in the first place, because our attention was elsewhere.

## State Synchronization

We successfully tracked the state of the digital robot, but we were unable to do
the same using the physical TurtleBot. We are still unsure about the root cause,
but a misconfigured system clock and digital TurtleBot were part of it. This
would be our number one priority for future improvement, as without this, it is
difficult to demonstrate the system's other features to stakeholders, as we
would only have digital demonstrations.

## Environmental & Object Interaction

We modelled environmental interaction in the digital environment by simulating
passengers leaving the bus stop (because they boarded the bus) whenever an agent
gets near. We could expand this by allowing for passengers to indicate at which
they want to get off and whether they want to transfer to another line from
there. This is not part of our requirements (yet), but it would make the system
much more useful to stakeholders, so we could consider it for future
improvement.

## Challenges with TurtleBot

As mentioned above we encountered the issue of syncing the Physical TurtleBot to 
the digital one.
The issue showed itself as only one "frame" of data in the RViz and Unity. 
We did not have time to fix it as replicating the issue was inconsistent due to the 
lack of understanding of what went wrong. 
On the last (filming) lab session it was Identified that it was probably issue with
clock desync. Unfortunately nobody knew how to solve it. So we did not end up utilizing
the digital robot. 

## Challenges with Development Environment

At first we wanted to use docker as we had people familiar with how slow VM was and how 
much space it takes on the storage. Unfortunately the docker installation encountered errors 
that were not solved. Asking tutors for help did not clarify the issue. So in week 3
we made a decision to switch to VM instead. This lead to significant loss of pace felt thought 
the course as we had 4 more lab sessions to implement physical part of our PoC. 

VM itself proved incredibly unstable and slow. With half of the team getting different results
for seemingly the same tests. In the end it was solved and we were able to use git. But any work on it proved
incredibly slow, with both Unity and RViz running at 1.5 fps.


## Challenges with RViz navigator

Navigator itself seems to sometimes do paths thought walls and/or too much turns.
It seems that robot overcompensates constantly which leads to it moving in circles.
We suspect that friction on one of the wheels is lower that on the other, which leads to different 
turning radius on both wheels. In addition we could did not have time to dig through the documentation 
to find where to make it "more confident" in its path finding. So all above problems seemed to dramatically increase
when we sized up the Unity scene appropriately. Including changing the robots
footprint in the folder. Explained [Here](/docs/Connecting%20Unity%20to%20Turtlebot.md#changing-the-size-of-the-robot-in-rviz). 

## Login details for lab laptops

Username: ubuntuhost
Password: otherwise-liverwurst-revolver-gamekeeper
