from scipy.io import loadmat
m = loadmat('./path.mat')
print m.keys()
print list(m['path'][0])