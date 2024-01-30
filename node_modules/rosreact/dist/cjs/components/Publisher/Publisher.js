"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null) for (var k in mod) if (k !== "default" && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
};
var __rest = (this && this.__rest) || function (s, e) {
    var t = {};
    for (var p in s) if (Object.prototype.hasOwnProperty.call(s, p) && e.indexOf(p) < 0)
        t[p] = s[p];
    if (s != null && typeof Object.getOwnPropertySymbols === "function")
        for (var i = 0, p = Object.getOwnPropertySymbols(s); i < p.length; i++) {
            if (e.indexOf(p[i]) < 0 && Object.prototype.propertyIsEnumerable.call(s, p[i]))
                t[p[i]] = s[p[i]];
        }
    return t;
};
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.Publisher = void 0;
var react_1 = __importStar(require("react"));
var prop_types_1 = __importDefault(require("prop-types"));
var RosConnection_1 = require("../RosConnection");
var Subscriber_1 = require("../Subscriber");
var Publisher = function (props) {
    var ros = (0, RosConnection_1.useRos)();
    var _a = (0, react_1.useState)(false), publisherTimer = _a[0], setPublisherTimer = _a[1];
    var topic = props.topic, messageType = props.messageType, throttleRate = props.throttleRate, latch = props.latch, queueLength = props.queueLength, queueSize = props.queueSize, otherProps = __rest(props, ["topic", "messageType", "throttleRate", "latch", "queueLength", "queueSize"]);
    var topicSettings = {
        topic: topic,
        messageType: messageType,
        throttleRate: throttleRate,
        latch: latch,
        queueLength: queueLength,
        queueSize: queueSize
    };
    var publisher = (0, Subscriber_1.getTopic)(ros, topicSettings);
    if (props.autoRepeat) {
        var rate = throttleRate || 1;
        var period_1 = Math.round(1000 / rate);
        (0, react_1.useEffect)(function () {
            var timer = setTimeout(function () {
                publisher.publish(props.message);
                setPublisherTimer(!publisherTimer);
            }, period_1);
            return function () { clearTimeout(timer); };
        }, [publisherTimer]);
    }
    else {
        (0, react_1.useEffect)(function () {
            publisher.publish(props.message);
        }, [props.message]);
    }
    (0, react_1.useEffect)(function () {
        return function () {
            publisher.unadvertise();
        };
    }, []);
    return react_1.default.createElement(react_1.Fragment, null);
};
exports.Publisher = Publisher;
exports.Publisher.propTypes = {
    topic: prop_types_1.default.string.isRequired,
    message: prop_types_1.default.object,
    messageType: prop_types_1.default.string.isRequired,
    throttleRate: prop_types_1.default.number,
    latch: prop_types_1.default.bool,
    queueLength: prop_types_1.default.number,
    queueSize: prop_types_1.default.number,
    autoRepeat: prop_types_1.default.bool,
};
//# sourceMappingURL=Publisher.js.map