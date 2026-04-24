# Feature Branch

*	Features
	*	Battery Management System
	*	Load Balancing System
	*	Newly Developed Deadlock Resolution Scheme
	*	Schedule based on Footprint
	

Build the environment using

`uv sync`

Then to activate the environment, run

`source .venv/bin/activate`

### How to run	

`python3 main.py`

### How to create a layout
* Edit the excel **config/file/layout_d.csv** file to build the layout.
* Run a file **notebook/map_to_graph.ipynb** to get a gml file which contains nodes and edges.
* Update filename **/config/research_layout.xml** to get a inbound, outbound and task station points.
* From using the .gml file to get a **/config/research_layout_path.xml** file which contains all the points and lines.
* Copy the contents of path file to a sample.xml file with Initial pose details. 
