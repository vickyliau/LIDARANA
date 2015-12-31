from liblas import file
from pyproj import Proj, transform
import fiona
from shapely.geometry import MultiPoint, Point, Polygon, MultiPolygon
from shapely.geometry import shape
from rtree import index
from osgeo import ogr, gdal, osr
import time
import numpy as num
from scipy.interpolate import griddata
import scipy

#get XYZ from the LIDAR data
f = file.File('E:/Dropbox/Project/campusGIS/UTD_data/UTD.las',mode='r')
#NAD 1983 UTM Zone 14N
NAD83N14=Proj("+proj=utm +zone=14 +ellps=GRS80 +datum=NAD83 +units=m +no_defs ", preserve_units = True)
#WGS84 Web Mercator (Auxiliary Sphere)
WGS84web=Proj("+proj=merc +lon_0=0 +k=1 +x_0=0 +y_0=0 +a=6378137 +b=6378137 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs ", preserve_units = True)

coorsX1=[]
coorsY1=[]
coorsZ1=[]
coorsX2=[]
coorsY2=[]
coorsZ2=[]
for p, q in zip(f, range(len(f))):
	if p.number_of_returns == 1: #the first return for the digital surface model
		coorsX1.append(p.x)
		coorsY1.append(p.y)
		coorsZ1.append(p.z)
	elif p.classification == 2: #classification for ground points
		coorsX2.append(p.x)
		coorsY2.append(p.y)
		coorsZ2.append(p.z)

#reprojection
web84_x1 = [0]*len(coorsX1)
web84_y1 = [0]*len(coorsX1)
web84_x2 = [0]*len(coorsX2)
web84_y2 = [0]*len(coorsX2)

for i in range(len(coorsX1)):
	a=transform(NAD83N14,WGS84web,coorsX1[i],coorsY1[i])
	web84_x1[i], web84_y1[i] = a[0], a[1]
for i in range(len(coorsX2)):
	b=transform(NAD83N14,WGS84web,coorsX2[i],coorsY2[i])
	web84_x2[i], web84_y2[i] = b[0], b[1]

web84_1=num.vstack((web84_y1,web84_x1)).T
web84_2=num.vstack((web84_y2,web84_x2)).T

#get image extents, based on the Convex hull algorithm
first_xmin, first_xmax=num.min(web84_x1),num.max(web84_x1)
first_ymin, first_ymax=num.min(web84_y1),num.max(web84_y1)
second_xmin, second_xmax=num.min(web84_x2),num.max(web84_x2)
second_ymin, second_ymax=num.min(web84_y2),num.max(web84_y2)

if first_xmax > second_xmax:
	B_xmax=first_xmax
else:
	B_xmax=second_xmax

if first_xmin > second_xmin:
	B_xmin=first_xmin
else:
	B_xmin=second_xmin

if first_ymax > second_ymax:
	B_ymax=first_ymax
else:
	B_ymax=second_ymax

if first_ymin > second_ymin:
	B_ymin=first_ymin
else:
	B_ymin=second_ymin

grid_x, grid_y = num.mgrid[B_ymin:B_ymax:1,B_xmin:B_xmax:1]
DSM=griddata(web84_1, num.array(coorsZ1), (grid_x, grid_y), method='nearest')
DTM=griddata(web84_2, num.array(coorsZ2), (grid_x, grid_y), method='nearest')
building=DSM-DTM

#gridsize
gridsize=1

#the heights of non-ground points, including buildings, trees and cars, since lidar classifications may not be accurate
driver = gdal.GetDriverByName('GTiff')
proj = osr.SpatialReference()
outfile = 'E:/Dropbox/Project/campusGIS/UTD_data/python/building_wgs84.tif'
dataset = driver.Create(outfile, building.shape[1], building.shape[0], 1, gdal.GDT_Float64, )
dataset.SetGeoTransform((B_xmin, gridsize,0,B_ymin,0,gridsize))
#wkt_projection='PROJCS["NAD_1983_UTM_Zone_14N",GEOGCS["GCS_North_American_1983",DATUM["D_North_American_1983",SPHEROID["GRS_1980",6378137,298.257222101]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]],PROJECTION["Transverse_Mercator"],PARAMETER["latitude_of_origin",0],PARAMETER["central_meridian",-99],PARAMETER["scale_factor",0.9996],PARAMETER["false_easting",500000],PARAMETER["false_northing",0],UNIT["Meter",1]]'
wkt_projection='PROJCS["WGS 84 / Pseudo-Mercator",GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]],PROJECTION["Mercator"],PARAMETER["central_meridian",0],PARAMETER["scale_factor",1],PARAMETER["false_easting",0],PARAMETER["false_northing",0],UNIT["Meter",1]]'
dataset.SetProjection(wkt_projection)
dataset.GetRasterBand(1).WriteArray(building)
dataset.FlushCache()

#Digital Terrain Model
driver = gdal.GetDriverByName('GTiff')
proj = osr.SpatialReference()
outfile = 'E:/Dropbox/Project/campusGIS/UTD_data/python/DTM_wgs84.tif'
dataset = driver.Create(outfile, DTM.shape[1], DTM.shape[0], 1, gdal.GDT_Float64, )
dataset.SetGeoTransform((B_xmin, gridsize,0,B_ymin,0,gridsize))
#wkt_projection='PROJCS["NAD_1983_UTM_Zone_14N",GEOGCS["GCS_North_American_1983",DATUM["D_North_American_1983",SPHEROID["GRS_1980",6378137,298.257222101]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]],PROJECTION["Transverse_Mercator"],PARAMETER["latitude_of_origin",0],PARAMETER["central_meridian",-99],PARAMETER["scale_factor",0.9996],PARAMETER["false_easting",500000],PARAMETER["false_northing",0],UNIT["Meter",1]]'
wkt_projection='PROJCS["WGS 84 / Pseudo-Mercator",GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]],PROJECTION["Mercator"],PARAMETER["central_meridian",0],PARAMETER["scale_factor",1],PARAMETER["false_easting",0],PARAMETER["false_northing",0],UNIT["Meter",1]]'
dataset.SetProjection(wkt_projection)
dataset.GetRasterBand(1).WriteArray(DTM)
dataset.FlushCache()

#Digital Surface Model
driver = gdal.GetDriverByName('GTiff')
proj = osr.SpatialReference()
outfile = 'E:/Dropbox/Project/campusGIS/UTD_data/python/DSM_wgs84.tif'
dataset = driver.Create(outfile, DSM.shape[1], DSM.shape[0], 1, gdal.GDT_Float64, )
dataset.SetGeoTransform((B_xmin, gridsize,0,B_ymin,0,gridsize))
#wkt_projection='PROJCS["NAD_1983_UTM_Zone_14N",GEOGCS["GCS_North_American_1983",DATUM["D_North_American_1983",SPHEROID["GRS_1980",6378137,298.257222101]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]],PROJECTION["Transverse_Mercator"],PARAMETER["latitude_of_origin",0],PARAMETER["central_meridian",-99],PARAMETER["scale_factor",0.9996],PARAMETER["false_easting",500000],PARAMETER["false_northing",0],UNIT["Meter",1]]'
wkt_projection='PROJCS["WGS 84 / Pseudo-Mercator",GEOGCS["GCS_WGS_1984",DATUM["D_WGS_1984",SPHEROID["WGS_1984",6378137,298.257223563]],PRIMEM["Greenwich",0],UNIT["Degree",0.017453292519943295]],PROJECTION["Mercator"],PARAMETER["central_meridian",0],PARAMETER["scale_factor",1],PARAMETER["false_easting",0],PARAMETER["false_northing",0],UNIT["Meter",1]]'
dataset.SetProjection(wkt_projection)
dataset.GetRasterBand(1).WriteArray(DSM)
dataset.FlushCache()

