import { useEffect, useState } from "react";
import { collection, onSnapshot } from "firebase/firestore";
import { db } from "@/firebase";
import type { AMRData, DroneData, ArmData, RobotFleetData } from "@/types";

export function useRobotFleet(): RobotFleetData {
  const [fleet, setFleet] = useState<RobotFleetData>({
    amr: null,
    drone: null,
    arm: null,
  });

  useEffect(() => {
    const unsub = onSnapshot(collection(db, "robots"), (snapshot) => {
      const next: RobotFleetData = { amr: null, drone: null, arm: null };
      snapshot.forEach((doc) => {
        const data = doc.data();
        if (doc.id === "amr_001") next.amr = data as AMRData;
        else if (doc.id === "drone_001") next.drone = data as DroneData;
        else if (doc.id === "m0609") next.arm = data as ArmData;
      });
      setFleet(next);
    });
    return unsub;
  }, []);

  return fleet;
}
