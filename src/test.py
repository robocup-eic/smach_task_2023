x = [
    "place_object",
    "move_to+grasp_object+place_object",
    "move_to+escort_people",
    "what_is_their_name",
    "move_to+ask_to_leave",
    "object_counting",
    "follow_people",
    "what_is_your_name",
    "move_to+follow_people",
    "grasp_object",

]

fuck = set()
for i in x:
    y = i.split("+")
    # fuck.add(i)
    for j in y:
        fuck.add(j)
    print(y)
print(fuck)