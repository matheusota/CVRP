import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt

with open('lb_ub_bcp.txt', 'r') as myfile:
	data = myfile.read()

data = data.split("\n")
lbs = []
ubs = []
its = []
k = 0
for l in data:
	l = l.replace("|", " ")
	l = l.split()
	
	if len(l) > 16:
		lb = l[15]
		ub = l[16]
		
		#if k % 10 == 0:
		if lb != "--":
			lbs.append(float(lb))
		else:
			lbs.append(0)
		if ub != "--":
			ubs.append(float(ub))
		else:
			ubs.append(2000)
		
		its.append(float(l[0].replace("s", "").replace("|", "")))
		k += 1
		#print(lb)
		#print(ub)

sns.set_style("darkgrid")

aux = {}
aux["valor da função objetivo"] = lbs
aux["tempo(s)"] = its
df = pd.DataFrame(aux)
print(df)
ax = sns.pointplot(x="tempo(s)", y="valor da função objetivo", data=df, color="darkcyan")

aux = {}
aux["valor da função objetivo"] = ubs
aux["tempo(s)"] = its
df = pd.DataFrame(aux)
print(df)
ax = sns.pointplot(x="tempo(s)", y="valor da função objetivo", data=df, color="limegreen")

plt.show()
