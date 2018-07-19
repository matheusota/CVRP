 #!/bin/bash
for filename in in/tcc/*.vrp; do
	./cvrp.e "-i" "in/tcc/$(basename "$filename")" "-o" "out/tcc/$(basename "$filename")" "-p" "-t" "2000"
done
