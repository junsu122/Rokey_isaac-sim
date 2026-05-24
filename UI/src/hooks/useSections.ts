import { useEffect, useState } from "react";
import { doc, collection, onSnapshot } from "firebase/firestore";
import { db } from "@/firebase";
import type { SectionData, PodData } from "@/types";

const SECTION_IDS = ["A", "B", "C"] as const;

export function useSections(): SectionData[] {
  const [sections, setSections] = useState<Record<string, Partial<SectionData>>>({});

  useEffect(() => {
    const unsubs: (() => void)[] = [];

    SECTION_IDS.forEach((id) => {
      // 섹션 문서 구독
      const secUnsub = onSnapshot(doc(db, "sections", id), (snap) => {
        if (!snap.exists()) return;
        setSections((prev) => ({
          ...prev,
          [id]: { ...prev[id], ...snap.data() },
        }));
      });

      // pods 서브컬렉션 구독
      const podsUnsub = onSnapshot(collection(db, "sections", id, "pods"), (snap) => {
        const pods: PodData[] = [];
        snap.forEach((d) => pods.push(d.data() as PodData));
        pods.sort((a, b) => a.pod_id.localeCompare(b.pod_id));
        setSections((prev) => ({
          ...prev,
          [id]: { ...prev[id], pods },
        }));
      });

      unsubs.push(secUnsub, podsUnsub);
    });

    return () => unsubs.forEach((u) => u());
  }, []);

  return SECTION_IDS
    .map((id) => sections[id])
    .filter((s): s is SectionData => !!s?.section_id);
}
