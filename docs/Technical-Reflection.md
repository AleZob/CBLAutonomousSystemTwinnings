# Technical Reflection

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

## Login details for lab laptops

Username: ubuntuhost
Password: otherwise-liverwurst-revolver-gamekeeper
