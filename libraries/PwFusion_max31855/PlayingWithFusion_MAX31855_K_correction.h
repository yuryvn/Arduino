PROGMEM const float K_voltage[670] = // T-type voltage array in 1 deg C increments, mV output
{-6.258,-6.256,-6.255,-6.253,-6.251,-6.248,-6.245,-6.242,-6.239,-6.236,-6.232,-6.228,-6.223,-6.219,
-6.214,-6.209,-6.204,-6.198,-6.193,-6.187,-6.18,-6.174,-6.167,-6.16,-6.153,-6.146,-6.138,-6.13,
-6.122,-6.114,-6.105,-6.096,-6.087,-6.078,-6.068,-6.059,-6.049,-6.038,-6.028,-6.017,-6.007,-5.996,
-5.985,-5.973,-5.962,-5.95,-5.938,-5.926,-5.914,-5.901,-5.888,-5.876,-5.863,-5.85,-5.836,-5.823,
-5.809,-5.795,-5.782,-5.767,-5.753,-5.739,-5.724,-5.71,-5.695,-5.68,-5.665,-5.65,-5.634,-5.619,
-5.603,-5.587,-5.571,-5.555,-5.539,-5.523,-5.506,-5.489,-5.473,-5.456,-5.439,-5.421,-5.404,-5.387,
-5.369,-5.351,-5.334,-5.316,-5.297,-5.279,-5.261,-5.242,-5.224,-5.205,-5.186,-5.167,-5.148,-5.128,
-5.109,-5.089,-5.07,-5.05,-5.03,-5.01,-4.989,-4.969,-4.949,-4.928,-4.907,-4.886,-4.865,-4.844,-4.823,
-4.802,-4.78,-4.759,-4.737,-4.715,-4.693,-4.671,-4.648,-4.626,-4.604,-4.581,-4.558,-4.535,-4.512,
-4.489,-4.466,-4.443,-4.419,-4.395,-4.372,-4.348,-4.324,-4.3,-4.275,-4.251,-4.226,-4.202,-4.177,
-4.152,-4.127,-4.102,-4.077,-4.052,-4.026,-4,-3.975,-3.949,-3.923,-3.897,-3.871,-3.844,-3.818,-3.791,
-3.765,-3.738,-3.711,-3.684,-3.657,-3.629,-3.602,-3.574,-3.547,-3.519,-3.491,-3.463,-3.435,-3.407,
-3.379,-3.35,-3.322,-3.293,-3.264,-3.235,-3.206,-3.177,-3.148,-3.118,-3.089,-3.059,-3.03,-3,-2.97,
-2.94,-2.91,-2.879,-2.849,-2.818,-2.788,-2.757,-2.726,-2.695,-2.664,-2.633,-2.602,-2.571,-2.539,-2.507,
-2.476,-2.444,-2.412,-2.38,-2.348,-2.316,-2.283,-2.251,-2.218,-2.186,-2.153,-2.12,-2.087,-2.054,-2.021,
-1.987,-1.954,-1.92,-1.887,-1.853,-1.819,-1.785,-1.751,-1.717,-1.683,-1.648,-1.614,-1.579,-1.545,-1.51,
-1.475,-1.44,-1.405,-1.37,-1.335,-1.299,-1.264,-1.228,-1.192,-1.157,-1.121,-1.085,-1.049,-1.013,-0.976,
-0.94,-0.904,-0.867,-0.83,-0.794,-0.757,-0.72,-0.683,-0.646,-0.608,-0.571,-0.534,-0.496,-0.459,-0.421,
-0.383,-0.345,-0.307,-0.269,-0.231,-0.193,-0.154,-0.116,-0.077,-0.039,0,0.039,0.078,0.117,0.156,0.195,
0.234,0.273,0.312,0.352,0.391,0.431,0.47,0.51,0.549,0.589,0.629,0.669,0.709,0.749,0.79,0.83,0.87,0.911,
0.951,0.992,1.033,1.074,1.114,1.155,1.196,1.238,1.279,1.32,1.362,1.403,1.445,1.486,1.528,1.57,1.612,
1.654,1.696,1.738,1.78,1.823,1.865,1.908,1.95,1.993,2.036,2.079,2.122,2.165,2.208,2.251,2.294,2.338,
2.381,2.425,2.468,2.512,2.556,2.6,2.643,2.687,2.732,2.776,2.82,2.864,2.909,2.953,2.998,3.043,3.087,
3.132,3.177,3.222,3.267,3.312,3.358,3.403,3.448,3.494,3.539,3.585,3.631,3.677,3.722,3.768,3.814,3.86,
3.907,3.953,3.999,4.046,4.092,4.138,4.185,4.232,4.279,4.325,4.372,4.419,4.466,4.513,4.561,4.608,4.655,
4.702,4.75,4.798,4.845,4.893,4.941,4.988,5.036,5.084,5.132,5.18,5.228,5.277,5.325,5.373,5.422,5.47,
5.519,5.567,5.616,5.665,5.714,5.763,5.812,5.861,5.91,5.959,6.008,6.057,6.107,6.156,6.206,6.255,6.305,
6.355,6.404,6.454,6.504,6.554,6.604,6.654,6.704,6.754,6.805,6.855,6.905,6.956,7.006,7.057,7.107,7.158,
7.209,7.26,7.31,7.361,7.412,7.463,7.515,7.566,7.617,7.668,7.72,7.771,7.823,7.874,7.926,7.977,8.029,
8.081,8.133,8.185,8.237,8.289,8.341,8.393,8.445,8.497,8.55,8.602,8.654,8.707,8.759,8.812,8.865,8.917,
8.97,9.023,9.076,9.129,9.182,9.235,9.288,9.341,9.395,9.448,9.501,9.555,9.608,9.662,9.715,9.769,9.822,
9.876,9.93,9.984,10.038,10.092,10.146,10.2,10.254,10.308,10.362,10.417,10.471,10.525,10.58,10.634,
10.689,10.743,10.798,10.853,10.907,10.962,11.017,11.072,11.127,11.182,11.237,11.292,11.347,11.403,
11.458,11.513,11.569,11.624,11.68,11.735,11.791,11.846,11.902,11.958,12.013,12.069,12.125,12.181,
12.237,12.293,12.349,12.405,12.461,12.518,12.574,12.63,12.687,12.743,12.799,12.856,12.912,12.969,
13.026,13.082,13.139,13.196,13.253,13.31,13.366,13.423,13.48,13.537,13.595,13.652,13.709,13.766,13.823,
13.881,13.938,13.995,14.053,14.11,14.168,14.226,14.283,14.341,14.399,14.456,14.514,14.572,14.63,14.688,
14.746,14.804,14.862,14.92,14.978,15.036,15.095,15.153,15.211,15.27,15.328,15.386,15.445,15.503,15.562,
15.621,15.679,15.738,15.797,15.856,15.914,15.973,16.032,16.091,16.15,16.209,16.268,16.327,16.387,16.446,
16.505,16.564,16.624,16.683,16.742,16.802,16.861,16.921,16.98,17.04,17.1,17.159,17.219,17.279,17.339,
17.399,17.458,17.518,17.578,17.638,17.698,17.759,17.819,17.879,17.939,17.999,18.06,18.12,18.18,18.241,
18.301,18.362,18.422,18.483,18.543,18.604,18.665,18.725,18.786,18.847,18.908,18.969,19.03,19.091,19.152,
19.213,19.274,19.335,19.396,19.457,19.518,19.579,19.641,19.702,19.763,19.825,19.886,19.947,20.009,20.07,
20.132,20.193,20.255,20.317,20.378,20.44,20.502,20.563,20.625,20.687,20.748,20.81};

const int16_t K_temp_offset = 270;	// value to subtract from voltage array index to get temperature