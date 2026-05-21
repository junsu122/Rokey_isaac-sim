import { useEffect, useState } from "react";
import { collection, onSnapshot, query, orderBy, limit } from "firebase/firestore";
import { db } from "@/firebase";
import type { ProductData, ItemData } from "@/types";

export interface InventoryData {
  products: ProductData[];
  items: ItemData[];
}

export function useInventory(): InventoryData {
  const [products, setProducts] = useState<ProductData[]>([]);
  const [items, setItems]       = useState<ItemData[]>([]);

  useEffect(() => {
    const unsubProducts = onSnapshot(collection(db, "products"), (snap) => {
      const list: ProductData[] = [];
      snap.forEach((doc) => list.push(doc.data() as ProductData));
      list.sort((a, b) => a.marker_id - b.marker_id);
      setProducts(list);
    });

    const itemsQ = query(
      collection(db, "items"),
      orderBy("detected_at", "desc"),
      limit(30)
    );
    const unsubItems = onSnapshot(itemsQ, (snap) => {
      const list: ItemData[] = [];
      snap.forEach((doc) => list.push(doc.data() as ItemData));
      setItems(list);
    });

    return () => { unsubProducts(); unsubItems(); };
  }, []);

  return { products, items };
}
