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
Spring 2020
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

This course is an introduction to research in model-based robotics. It focuses on dynamics and control of
robot manipulators, mobile robotics, grasping and manipulation, and soft robotics.

The course is a sequel to EECS C106A/Bioengineering C106A and EECS C206A which covers the mathematical fundamentals of robotics including kinematics and dynamics. This course will delve deeper into several areas of robotics which are currently of research interest. Concepts will include manipulator dynamics and control, nonholonomic steering of mobile robots, path planning, grasp modeling with friction, grasp planning, and soft robotics.