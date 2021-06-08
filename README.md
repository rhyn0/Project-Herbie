# Welcome to Herbie Skeleton Software Base

## Created by Ryan Ozawa for Cal Poly Senior Project

These commands have been updated from time-based thread sleeping commands to distance calculating via encoder counts.
This is customized to work with the physical layout of the Herbie Skeleton, see any .pdf for those measurements and customizations

### Setup

- To use need to have Python 3.7 or later installed. 
- Other submodules are needed such as ```pip install serial threading```

Was made to run on a RaspberryPi 4B with 8GB of Memory.

To view documentation on functions inside commands.py and motor.py run:
```python 
python3 inspector.py
```
And then open Manual.txt, this will give you the most updated documentation on each function.

### Getting Started

The following command will open the command shell for the Rover, and need to input commands according to DESCRIPTIONS.txt.
```python
python3 start.py
```
