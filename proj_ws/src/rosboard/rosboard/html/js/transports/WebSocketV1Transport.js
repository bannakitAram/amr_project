class WebSocketV1Transport {
    constructor({path, onOpen, onClose, onRosMsg, onTopics, onSystem, onAuthResult}) {
      this.path = path;
      this.onOpen = onOpen ? onOpen.bind(this) : null;
      this.onClose = onClose ? onClose.bind(this) : null;
      this.onMsg = onRosMsg ? onRosMsg.bind(this) : null;
      this.onTopics = onTopics ? onTopics.bind(this) : null;
      this.onSystem = onSystem ? onSystem.bind(this) : null;
      this.onAuthResult = onAuthResult ? onAuthResult.bind(this) : null;
      this.ws = null;
    }
  
    connect() {
      var protocolPrefix = (window.location.protocol === 'https:') ? 'wss:' : 'ws:';
      let abspath = protocolPrefix + '//' + location.host + this.path;
  
      let that = this;
  
      this.ws = new WebSocket(abspath);
  
      this.ws.onopen = function(){
        console.log("connected");
        if(that.onOpen) that.onOpen(that);
      }
      
      this.ws.onclose = function(){
        console.log("disconnected");
        if(that.onClose) that.onClose(that);
      }
  
      this.ws.onmessage = function(wsmsg) {
        let data = [];
        try {
          // try fast native parser
          data = JSON.parse(wsmsg.data);
        } catch(e) {
          // Python may have included Infinity, -Infinity, NaN
          // fall back to a JSON5 parsers which will deal with these well but is almost 50X slower in Chrome
          data = JSON5.parse(wsmsg.data);
        }

        let wsMsgType = data[0];
  
        if(wsMsgType === WebSocketV1Transport.MSG_PING) {
          this.send(JSON.stringify([WebSocketV1Transport.MSG_PONG, {
            [WebSocketV1Transport.PONG_SEQ]: data[1][WebSocketV1Transport.PING_SEQ],
            [WebSocketV1Transport.PONG_TIME]: Date.now(),
          }]));
        }
        else if(wsMsgType === WebSocketV1Transport.MSG_MSG && that.onMsg) that.onMsg(data[1]);
        else if(wsMsgType === WebSocketV1Transport.MSG_TOPICS && that.onTopics) that.onTopics(data[1]);
        else if(wsMsgType === WebSocketV1Transport.MSG_SYSTEM && that.onSystem) that.onSystem(data[1]);
        else if(wsMsgType === WebSocketV1Transport.MSG_AUTH_RESULT && that.onAuthResult) that.onAuthResult(data[1]);
        else console.log("received unknown message: " + wsmsg.data);
      }
    }
  
    isConnected() {
      return (this.ws && this.ws.readyState === this.ws.OPEN);
    }
  
    subscribe({topicName, maxUpdateRate = 24.0}) {
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_SUB, {topicName: topicName, maxUpdateRate: maxUpdateRate}]));
    }

    unsubscribe({topicName}) {
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_UNSUB, {topicName: topicName}]));
    }

    teleop({linear, angular}) {
      if (!this.isConnected()) return;
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_TELEOP, {linear: linear, angular: angular}]));
    }

    authenticate({code}) {
      if (!this.isConnected()) return;
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_AUTH, {code: code}]));
    }

    navGoal({x, y, theta}) {
      if (!this.isConnected()) return;
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_NAV_GOAL, {x: x, y: y, theta: theta}]));
    }

    navPath({points}) {
      if (!this.isConnected()) return;
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_NAV_PATH, {points: points}]));
    }

    navCancel() {
      if (!this.isConnected()) return;
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_NAV_CANCEL, {}]));
    }

    teleopCfg({maxLinear}) {
      if (!this.isConnected()) return;
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_TELEOP_CFG, {max_linear: maxLinear}]));
    }

    mappingCmd({action, name}) {
      if (!this.isConnected()) return;
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_MAPPING, {action: action, name: name || ""}]));
    }

    bagCmd({action, topics}) {
      if (!this.isConnected()) return;
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_BAG, {action: action, topics: topics || []}]));
    }

    initialPose({x, y, theta}) {
      if (!this.isConnected()) return;
      this.ws.send(JSON.stringify([WebSocketV1Transport.MSG_INITIAL_POSE, {x: x, y: y, theta: theta}]));
    }
  }

  WebSocketV1Transport.MSG_PING = "p";
  WebSocketV1Transport.MSG_PONG = "q";
  WebSocketV1Transport.MSG_MSG = "m";
  WebSocketV1Transport.MSG_TOPICS = "t";
  WebSocketV1Transport.MSG_SUB = "s";
  WebSocketV1Transport.MSG_SYSTEM = "y";
  WebSocketV1Transport.MSG_UNSUB = "u";
  WebSocketV1Transport.MSG_TELEOP = "v";
  WebSocketV1Transport.MSG_AUTH = "a";
  WebSocketV1Transport.MSG_AUTH_RESULT = "r";
  WebSocketV1Transport.MSG_NAV_GOAL = "g";
  WebSocketV1Transport.MSG_NAV_PATH = "w";
  WebSocketV1Transport.MSG_NAV_CANCEL = "c";
  WebSocketV1Transport.MSG_INITIAL_POSE = "i";
  WebSocketV1Transport.MSG_MAPPING = "k";
  WebSocketV1Transport.MSG_TELEOP_CFG = "x";
  WebSocketV1Transport.MSG_BAG = "b";

  WebSocketV1Transport.PING_SEQ= "s";
  WebSocketV1Transport.PONG_SEQ = "s";
  WebSocketV1Transport.PONG_TIME = "t";