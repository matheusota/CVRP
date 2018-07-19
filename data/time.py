time_bc = 0
time_bcp = 0
time_bcp_fuka = 0

with open('time_bc', 'r') as f:
	for l in f:
		l = l.replace(" ", "")
		l = l.replace("\\\\\hline\n",  "")
		l = l.split("&")
		time_bc += float(l[-1])

with open('time_bcp', 'r') as f:
	for l in f:
		l = l.replace(" ", "")
		l = l.replace("\\\\\hline\n",  "")
		l = l.split("&")
		time_bcp += float(l[-1])


with open('time_bcp_fuka', 'r') as f:
	for l in f:
		l = l.replace(" ", "")
		l = l.replace("\\\\\hline\n",  "")
		l = l.split("&")
		time_bcp_fuka += float(l[-1])

print(time_bc)
print(time_bcp)
print(time_bcp_fuka)
