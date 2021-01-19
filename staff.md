---
layout: page
title: Staff
nav_order: 5
description: A listing of all the course staff members.
---

## Instructors

<div class="role">
  {% assign instructors = site.staffers | where: 'role', 'Instructor' %}
  {% for staffer in instructors %}
  {{ staffer }}
  {% endfor %}
</div>

## Teaching Assistants

<div class="role">
  {% assign teaching_assistants = site.staffers | where: 'role', 'Head TA' %}
  {% for staffer in teaching_assistants %}
  {{ staffer }}
  {% endfor %}
  {% assign teaching_assistants = site.staffers | where: 'role', 'Content TA' %}
  {% for staffer in teaching_assistants %}
  {{ staffer }}
  {% endfor %}
  {% assign teaching_assistants = site.staffers | where: 'role', 'Teaching Assistant' %}
  {% for staffer in teaching_assistants %}
  {{ staffer }}
  {% endfor %}
</div>

## Moral Support
<div class="role">
  {% assign ms = site.staffers | where: 'role', 'Moral Support' %}
  {% for staffer in ms %}
  {{ staffer }}
  {% endfor %}
</div>