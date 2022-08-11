---
layout: post
title:  "Scaling Multi-Agent Reinforcement Learning via State Upsampling"
date:   2022-06-19 00:00:00 +00:00
image: /images/scalingmarl.png
categories: research
author: ""
authors: "<strong>Luis Pimentel*</strong>, Rohan Paleja*, Zheyuan Wang, Esmaeil Seraj, James Pagan, and Matthew Gombolay"
venue: In Proc. <a href="https://www.gatech.edu/" style="text-decoration:none;font-weight:bold">RSS 2022 Workshop on Scaling Robot Learning (RSS22-SRL).</a>
paper: /pdfs/RSS22_SRL_Workshop_Paper.pdf
poster: /pdfs/RSS22-SRL-Workshop-Poster.pdf
---

We consider the problem of scaling Multi-Agent Reinforcement Learning (MARL) algorithms toward larger environments and team sizes. While it is possible to learn a MARL-synthesized policy on these larger problems from scratch, training is difficult as the joint state-action space is much larger. In this paper, we propose a transfer learning method that accelerates the training performance in such high-dimensional tasks with increased complexity. Our method upsamples an agent’s state representation in a smaller, less challenging, source task in order to pre-train a target policy for a larger, more challenging, target task. By transferring the policy after pre-training and continuing MARL in the target domain, the information learned within the source task enables higher performance within the target task in significantly less time than training from scratch. 