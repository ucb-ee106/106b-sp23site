---
layout: home
title: Home
nav_order: 0
description: >-
    Course website for EECS 106/206B Spring 2021
---
<!-- <div class="parallax-window" data-parallax="scroll" data-image-src="/assets/background.png" data-speed="0.1">/div> -->
# EECS C106B/206B | Robotic Manipulation and Interaction
{: .mb-2 }
Spring 2021
{: .mb-0 .fs-6 .text-grey-dk-200 }

{% assign instructors = site.staffers | where: 'role', 'Instructor' %}
<div class="role">
  {% for staffer in instructors %}
  {{ staffer }}
  {% endfor %}
</div>

{% if site.announcements %}
{{ site.announcements.last }}
<a href="{{ site.baseurl }}/announcements" class="btn btn-outline fs-3">
  All Announcements
</a>
{% endif %}

## Navigating the Website

All assignment due dates can be found in the *Policies* tab under *Due Dates*.

Looking for the weekly lab, discussion, lecture, or office hours schedule? Check out the *Schedule* tab!

Looking for the semester plan, discussion worksheets, project assignments, or homework assignments? Check out the *Course Calendar* tab!

Have a question about our course policies? Check out the *Policies* tab!

Looking for a TA or professor's email? Check out the *Staff* tab!

Looking for resources for projects, homeworks, and lecture? Check out the *Resources* tab!

## Course Description

This course is an introduction to advanced topics and research in robotics and intelligent machines. The course is a sequel to EECS/Bioengineering/ME C106A and EECS C206A which covers the mathematical fundamentals of robotics including kinematics, dynamics and control as well as an introduction to path planning, obstacle avoidance, and computer vision This course will present several areas of robotics and active vision, at a deeper level and informed by current research. Concepts will include the review at an advanced level of robot control, the kinematics, dynamics and control of multi-fingered hands, grasping and manipulation of objects, mobile robots: including non-holonomic motion planning and control, path planning, Simultaneous Localization And Mapping (SLAM), and active vision. Additional research topics to be covered at the instructor’s discretion include: locomotion and walking robots, Unmanned Aerial Vehicles, soft robotics, and Augmented/Virtual Reality.