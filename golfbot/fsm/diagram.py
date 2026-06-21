import json
import io, requests
from os import linesep as ln
from pathlib import Path
from PIL import Image as im
import numpy as np
import cv2
import base64



if __name__ == '__main__':
    fsm = json.load(io.open(str(Path(__file__).parent /  "fsm.json")))

    a = ["1", "2", "3"]
    for i,a  in enumerate(a):
        pass
    states: dict[str, tuple[int, str]] = {(name): (i, state['name']) for i, (name, state) in enumerate(fsm['states'].items())}
    stateTransitions = [(name, transition) for name, transition in fsm['stateTransitions'].items()]    

    output = f"---{ln}title: FSM{ln}---{ln}stateDiagram-v2{ln}\tdirection LR{ln}"

    for stateName, (id, title) in states.items():
        output += f"\tstate \"{title}\" as s{id}{ln}"

    output += ln

    for fromState, transitions in stateTransitions:
        for transition in transitions:
            title, toState = transition['name'], transition['nextState']
            output += f"\ts{states[fromState][0]} --> s{states[toState][0]}: {title}{ln}"
    
    graphbytes = output.encode("utf8")
    base64_bytes = base64.urlsafe_b64encode(graphbytes)
    base64_string = base64_bytes.decode("ascii")
    # I could not find a single mermaid renderer for python,
    # so it seems that the only option is to use this.
    img = im.open(io.BytesIO(requests.get('https://mermaid.ink/img/' + base64_string).content))
    open_cv_image = np.array(img)
    cv2.imshow("image", open_cv_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()