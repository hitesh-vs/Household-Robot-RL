# High to Low Level Task Planning and Execution using RL 

This project presents a framework for mobile manipulators using Hierarchical Reinforcement Learning (HRL) and Reward Shaping to tackle complex tasks efficiently. Intrinsic Curiosity fosters self-driven exploration, while Unity ML Agents enable a proof-of-concept for navigation and object manipulation in unknown environments. Future work aims to integrate large language models (LLMs) for task decomposition and enhanced automation.

The specific problem we aim to solve through this project is for a **mobile manipulator to learn to solve the problem of cleaning a table in a room autonomously**. For this purpose, the robot first needs to navigate to the location of the table in the room, then pick up the trash on the table and then navigate to the location of the trash can. We carried out the implementation in three phases:

1. Starting from the low level tasks, we created an RL agent to learn to navigate to a target location avoiding obstacles
2. After Reaching the location, another agent is created to learn to pick the trash
3. Once the agents for navigation and picking have learnt the optimal policies, they are integrated through the concepts of Heirarchical RL.


The complete implementation of the above phases and the results we obtained are depicted in this presentation : [Link](https://rltaskplanner.my.canva.site/plan)
