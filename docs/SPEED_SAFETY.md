# Speed Safety Score

This page documents the methods for obtaining a safety score for speed. The `EGO_SPEED` proposition works by collecting a range for speed values for the test time frame. A safety score is calculated for each speed profile and an average speed score is calculated.

# Content

# General Concept

The safety score quantifies the safety related to the velocity profile of the `EGO_VEHICLE`. For this purpose, velocity ranges, absolute velocity tolerance and guard conditions must be defined. Here the velocity acts as an `EGO_BEHAVIOR`. Before quantification, the consequences of exceeding the absolute velocity tolerance must be defined. The tolerance sets an upper limit on the behavior above which the system cannot be allowed to operate. A safety score must operate within this allowed bound. 

-> Update can be done even if the test is not failed
-> Provides a better understanding of the systems abilities beyond a boolean Pass/Fail method.
-> Level of danger in the case that a system barely passes

# Safety Classification

-> Talk about the different scales
-> Why is it beneficial to have different scales

# Safety Scoring Method

-> Bounding Conditions in different Classes
-> Guard Conditions

# Safety Score to Grading

-> German and American System