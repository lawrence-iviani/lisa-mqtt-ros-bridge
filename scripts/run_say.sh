#!/bin/bash

sleep_btw=4
rosservice call /lisa/service/say "context_id: 'ctxt_1' 
text: 'This is the first message, can be enqueued and can be discarded' 
canbeenqued: true 
canbediscarded: true"

sleep $sleep_btw
rosservice call /lisa/service/say "context_id: 'ctxt_2' 
text: 'This is the second message, If you hear this message, it is bad.  cannot be enqueued and can be discarded' 
canbeenqued: false 
canbediscarded: true"

sleep $sleep_btw
rosservice call /lisa/service/say "context_id: 'ctxt_3' 
text: 'This is the third message, cannot be enqueued and cannot be discarded' 
canbeenqued: false 
canbediscarded: false"

sleep $sleep_btw
rosservice call /lisa/service/say "context_id: 'ctxt_4' 
text: 'This is the forth message, If you hear this message, it is bad' 
canbeenqued: false 
canbediscarded: true"

sleep $sleep_btw
rosservice call /lisa/service/say "context_id: 'ctxt_5' 
text: 'This is the fifth message and  can be enqueued and cannot be discarded' 
canbeenqued: true
canbediscarded: false"

