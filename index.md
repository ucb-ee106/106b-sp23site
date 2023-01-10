---
layout: home
title: Home
nav_order: 0
description: >-
    Course website for EECS 106/206B Spring 2022
---
<!-- <div class="parallax-window" data-parallax="scroll" data-image-src="/assets/background.png" data-speed="0.1">/div> -->
# EECS C106B/206B | Robotic Manipulation and Interaction
{: .mb-2 }
Spring 2022
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

# Course Calendar

{% for module in site.modules %}
{{ module }}
{% endfor %}
