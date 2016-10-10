What are Isochrone Maps?
------------------------

Recently valhalla has gained the ability to return these amazing structures called isochrones. What's an isochrone? The word is a combination of two greek roots `iso` meaning equal and `chrono` meaning time. So indeed, an isochrone is a structure representing equal time. In our case it's a line that reprsents constant travel time about a given location. One can think of isochrone maps as somehwat similar to the familiar topographic maps except that instead of lines of constant height, lines are of constant travel time are depicted. For this reason other terms common in topography apply such as contours or isolines.

![Isochrone Map](images/isochrone.png "Melbourne Driving Isochrone Map")
In this image the green, yellow, orange and red contour lines represent 15, 30, 45 and 60 minutes of driving time respectively.

How are Isochrone Maps Useful?
------------------------------

Isochrone maps can be used to make informed decisions about travel at both an individual level and en masse. You can get quantitative answeres to questions like:

 * What are our lunch options within 5 minutes from here?
 * How much of the city lives within walking range of public transit?
 * What would adding/removing this road/bus stop/bridge do to travel times?
 * Where can I find housing that still has a reasonable commute to the office?

In other words planning departments of DOTs all the way down to consumer applications will have use-cases that call for such an isochrone map service. 

Where is it?
------------

You can find the [API documentation here](https://github.com/valhalla/valhalla-docs/blob/master/isochrones/api-reference.md) and you see a live demo of it right [here](https://valhalla.github.io/demos/isochrone/).
