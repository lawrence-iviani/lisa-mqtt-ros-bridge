#!/bin/bash

sleep_btw=4
rosservice call /lisa/service/interact "context_id: 'ctxt_1' 
text: 'This is the firsti time i ask for intention , can be enqueued and can be discarded' 
canbeenqued: true 
canbediscarded: true"

sleep $sleep_btw
rosservice call /lisa/service/interact "context_id: 'ctxt_2' 
text: 'If you hear this message, it is bad.' 
canbeenqued: false 
canbediscarded: true"

sleep $sleep_btw
rosservice call /lisa/service/interact "context_id: 'ctxt_3' 
text: 'This is the second time i ask for intention, cannot be enqueued and cannot be discarded' 
canbeenqued: false 
canbediscarded: false"

sleep $sleep_btw
rosservice call /lisa/service/interact "context_id: 'ctxt_4' 
text: 'If you hear this message, it is bad' 
canbeenqued: false 
canbediscarded: true"

sleep $sleep_btw
rosservice call /lisa/service/interact "context_id: 'ctxt_5' 
text: 'This is the third time i ask for intention and  can be enqueued and cannot be discarded' 
canbeenqued: true
canbediscarded: false"



