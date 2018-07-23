import pandas as pd, numpy as np
from matplotlib import pyplot

one44 = np.load('144.0.npy')
goodpart = one44[0]
goodx = goodpart[:,2] > -65
goody = ((goodpart[:,1] < 93) & (goodpart[:,1] > 0))
goodpart = goodpart[goodx & goody, :]
goodpart = pd.DataFrame(goodpart)
goodpart = goodpart.sort_values(by=2, ascending=False)
pyplot.plot(goodpart[1], goodpart[2])

tether = pd.DataFrame(np.empty(goodpart.shape))
tether[:] = np.nan
droplist= []
last_tether_point = goodpart.iloc[0,:]
#Walk the wall downwards
for n, row in goodpart.iterrows():
    #if Y val decreases then we roll along
    if row[1] <= last_tether_point[1]:
        last_tether_point = row
        tether.iloc[n, :] = last_tether_point
    else:
        droplist.append(n)
tether.drop(droplist, inplace=True)
tether = tether.sort_values(by=2, ascending=False)
pyplot.plot(tether[1], tether[2], ':k')


pyplot.show()