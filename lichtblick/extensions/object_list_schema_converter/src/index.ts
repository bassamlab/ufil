import { ExtensionContext, Immutable, MessageEvent } from "@foxglove/extension";

/**
 * Minimal structural typing for UFIL ObjectList
 * (no ROS imports, no codegen required)
 */
type ObjectList = {
  header: {
    frame_id: string;
    stamp?: unknown;
  };
  objects: UfilObject[];
};

type UfilObject = {
  id: number;
  state: {
    state: { 
      x: number;
      y: number;
      z: number;
      yaw: number;
    }
  };
  dimension: {
    dimension: {
      length: number;
      width: number;
      height: number;
    }
  };
};

function yawToQuaternion(y: number) {
  const yaw = y;
  return {
    x: 0,
    y: 0,
    z: Math.sin(yaw * 0.5),
    w: Math.cos(yaw * 0.5),
  };
}

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerMessageConverter({
    type: "schema",
    fromSchemaName: "ufil_msgs/msg/ObjectList",
    toSchemaName: "foxglove.SceneUpdate",
    converter: (msg: ObjectList, event: Immutable<MessageEvent<ObjectList>>) => {
      const frameId = msg.header.frame_id || "map";

      // delete ALL
      const deletions = [
        {
          type: 1
        }
      ];

      const entities = msg.objects.map((obj) => ({
        id: `${obj.id}`,
        frame_id: frameId,
        timestamp: event.publishTime,
        lifetime: { sec: 1 }, 
        cubes: [
          {
            size: {
              x: obj.dimension.dimension.length,
              y: obj.dimension.dimension.width,
              z: obj.dimension.dimension.height,
            },
            pose: {
              position: {
                x: obj.state.state.x,
                y: obj.state.state.y,
                z: obj.dimension.dimension.height * 0.5,
              },
              orientation: yawToQuaternion(obj.state.state.yaw)
            },
          },
        ],
      }));

      return {
        deletions,
        entities,
      };
    },
  });
}
