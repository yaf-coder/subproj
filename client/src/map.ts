import maplibregl, { Map, Marker, LngLatLike } from "maplibre-gl";
import type { Plan } from "./api";

const STYLE = {
  version: 8 as const,
  sources: {
    osm: {
      type: "raster" as const,
      tiles: ["https://tile.openstreetmap.org/{z}/{x}/{y}.png"],
      tileSize: 256,
      attribution: "&copy; OpenStreetMap contributors",
    },
  },
  layers: [{ id: "osm", type: "raster" as const, source: "osm" }],
};

export interface MapHandle {
  map: Map;
  setStart(ll: LngLatLike | null): void;
  setGoal(ll: LngLatLike | null): void;
  setRoute(plan: Plan | null): void;
  setSub(lng: number, lat: number, heading_deg: number): void;
  appendTrack(lng: number, lat: number): void;
  clearTrack(): void;
}

export function initMap(container: HTMLElement, onClick: (lng: number, lat: number) => void): MapHandle {
  const map = new maplibregl.Map({
    container,
    style: STYLE,
    center: [-121.89, 36.61],
    zoom: 13,
    pitch: 0,
  });

  let startMarker: Marker | null = null;
  let goalMarker: Marker | null = null;
  let subMarker: Marker | null = null;
  const trackCoords: [number, number][] = [];

  map.on("load", () => {
    map.addSource("route", { type: "geojson", data: emptyLine() });
    map.addLayer({
      id: "route-line",
      type: "line",
      source: "route",
      paint: {
        "line-color": "#4cc4ff",
        "line-width": 3,
        "line-dasharray": [2, 2],
      },
    });
    map.addSource("track", { type: "geojson", data: emptyLine() });
    map.addLayer({
      id: "track-line",
      type: "line",
      source: "track",
      paint: { "line-color": "#ffa000", "line-width": 2 },
    });
  });

  map.on("click", (e) => onClick(e.lngLat.lng, e.lngLat.lat));

  function emptyLine(): GeoJSON.FeatureCollection {
    return { type: "FeatureCollection", features: [] };
  }
  function lineFC(coords: [number, number][]): GeoJSON.FeatureCollection {
    return {
      type: "FeatureCollection",
      features: [{
        type: "Feature",
        properties: {},
        geometry: { type: "LineString", coordinates: coords },
      }],
    };
  }

  function colorPin(color: string): HTMLElement {
    const el = document.createElement("div");
    el.style.cssText = `
      width: 16px; height: 16px; border-radius: 50%;
      background: ${color}; border: 2px solid #001827;
      box-shadow: 0 0 0 2px ${color}88;
    `;
    return el;
  }

  function subPin(): HTMLElement {
    const el = document.createElement("div");
    el.style.cssText = `
      width: 0; height: 0;
      border-left: 8px solid transparent;
      border-right: 8px solid transparent;
      border-bottom: 18px solid #4cc4ff;
      filter: drop-shadow(0 0 4px #4cc4ff88);
      transform-origin: 50% 70%;
    `;
    return el;
  }

  return {
    map,
    setStart(ll) {
      startMarker?.remove();
      startMarker = ll ? new maplibregl.Marker({ element: colorPin("#4cc4ff") }).setLngLat(ll).addTo(map) : null;
    },
    setGoal(ll) {
      goalMarker?.remove();
      goalMarker = ll ? new maplibregl.Marker({ element: colorPin("#ffa000") }).setLngLat(ll).addTo(map) : null;
    },
    setRoute(plan) {
      const src = map.getSource("route") as maplibregl.GeoJSONSource | undefined;
      if (!src) return;
      if (!plan) { src.setData(emptyLine()); return; }
      const coords: [number, number][] = plan.waypoints.map(w => [w.lon, w.lat]);
      src.setData(lineFC(coords));
    },
    setSub(lng, lat, heading_deg) {
      if (!subMarker) {
        subMarker = new maplibregl.Marker({ element: subPin(), rotationAlignment: "map" }).setLngLat([lng, lat]).addTo(map);
      } else {
        subMarker.setLngLat([lng, lat]);
      }
      // Heading in map: 0 deg = north, increasing clockwise. Our heading_deg is from east, CCW.
      // Convert: map_bearing = 90 - heading_deg.
      const map_bearing = 90 - heading_deg;
      subMarker.setRotation(map_bearing);
    },
    appendTrack(lng, lat) {
      trackCoords.push([lng, lat]);
      const src = map.getSource("track") as maplibregl.GeoJSONSource | undefined;
      if (src) src.setData(lineFC(trackCoords));
    },
    clearTrack() {
      trackCoords.length = 0;
      const src = map.getSource("track") as maplibregl.GeoJSONSource | undefined;
      if (src) src.setData(emptyLine());
    },
  };
}
