import re
import numpy as np

initialised = False


def initializeStr():
    global s
    # find beginning of the most recent data cluster
    startFrom = s.rfind('<', 0, len(s))
    if (startFrom > -1):
        s = s[startFrom:]
        print 'clipped: {}'.format(s)
        initialised = True
    else:
        s = '' # didnt a data start point, clear it out

def processStr():
    global s
    if not initialised:
        initializeStr()
    found = re.search('<(.*)>', s)
    if found is not None:
        print('found a full match')
        return found.group(1)
    else:
        return None

s = 'lkjh<12.34,56.78,90.33333>asdf'
result = processStr()
if result is not None:
    remainder = s[len(result)+2:]
    split = result.split(',')
    result = np.array(map(float, split))
    for i in result:
        print type(i)
    print 'result: {}, remainder: {}, s: {}'.format(result, remainder, s)
    s = remainder
else:
    print "No match found :("
